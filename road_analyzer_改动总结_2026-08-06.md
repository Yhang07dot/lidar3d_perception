# Road Analyzer 路沿检测改动总结 (2026-08-06)

## 概述

对 `road_analyzer.py` 进行三轮迭代改动，最终实现**纯 nonground 单源、前向双侧车道线检测**，解决青色边界线交叉、绕到车身后方和不贴合道路两侧障碍物的问题。

---

## 第一轮：双源优化（ground + nonground）

### 问题
- 青色路沿线交叉穿过车身：全局 RANSAC 直线拟合把弯道拉成斜线
- 边界没贴在绿色路沿障碍物上：nonground 取的是 band 内**最远**点
- 车身附近有杂乱尖刺：未过滤靠近中心线的候选点

### 改动
| 改动项 | 说明 |
|--------|------|
| `_nonground_kerb_point` | nonground 候选改为取**最近**的地面外侧点（障碍物内侧面） |
| `_extract_boundaries_dual` | 优先用 nonground 路沿，回退到 ground edge；增加 `\|y\| < min_lateral` 过滤 |
| `_smooth_boundary_polar` | 取消全局直线拟合，改为按角度排序的极坐标局部平滑 |
| 参数 | 新增 `min_lateral=0.5`；移除 `ransac_dist_thr` |

---

## 第二轮：移除 ground 订阅

### 问题
- 青色线仍然形成大三角交叉
- 用户指出：既然 kmcb 障碍物已是 nonground，不应使用 ground 点云干扰

### 改动
| 改动项 | 说明 |
|--------|------|
| 移除此/`patchworkpp/ground` 订阅 | 只保留 `/patchworkpp/nonground` |
| `_on_ground` 回调 | 删除 |
| `_ground_edge_per_bin` / `_ground_edge_xy` | 删除 |
| `_nonground_kerb_point` → `_nearest_nonground_per_bin` | 不再依赖 ground edge 距离，直接取每 bin 最近 nonground 点 |
| `_extract_boundaries_dual` → `_extract_boundaries_nonground` | 单源提取，保留 `min_lateral` 过滤 |
| 移除 `gap_threshold` 参数 | 不再使用 |

---

## 第三轮：纵向双侧车道线生成（当前方案）

### 问题
- 角度 bin 覆盖 360°，候选点会包含车侧和车后障碍物；按极角连线时容易闭合成大三角
- 定时发布时会把同一帧反复写入缓存，缓存点与实时点的顺序混杂，进一步造成折返连线
- 左右边界按相同极角配对无法正确生成道路中心线

### 改动
| 改动项 | 说明 |
|--------|------|
| `_extract_lane_boundaries` | 改为沿车辆前进方向（`x`）分 bin；每个 bin 同时选择左右障碍物的道路内侧面 |
| 配对约束 | 只保留道路宽度在 `min_road_width` ～ `max_road_width` 内、且接近中位宽度的左右候选对 |
| 连续性约束 | 保留宽度一致的稀疏纵向支撑，仅剔除局部横向突跳点；适配 16 线雷达两侧障碍物不在同一 x-bin 的情况 |
| `_smooth_lane_track` | 对按 `x` 排序的每侧轨迹进行中值平滑；`LINE_STRIP` 始终按前向顺序发布 |
| 缓存合并 | 每个输入点云帧最多写入一次世界坐标缓存；合并时按 `x` bin 去重并过滤车后缓存点 |
| `_compute_centerline` | 改为在左右共同的 `x` 范围内插值求中点 |

---

## 当前算法流程

```
/patchworkpp/nonground (PointCloud2)
    │
    ├── _pc2_to_xyz() → (N,3) numpy array
    │
    ├── TF: source_frame → base_link
    │
    ├── _extract_lane_boundaries()
    │       │
    │       ├── 只保留前方 `min_forward` ～ `max_forward` 的点
    │       ├── 每 `forward_bin_size` 米的 x-bin 提取左右障碍物内侧面
    │       ├── 道路宽度和横向连续性约束 → 成对左右边界
    │       └── 过滤中央孤立障碍物和车后点
    │
    ├── _smooth_lane_track(window=5) → 按 x 排序的左右轨迹
    │
    └── 世界坐标缓存去重后发布 /lidar/road_boundary_markers (LINE_STRIP ×2)
        发布 /lidar/centerline (Path)
```

---

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_forward` | 1.0 | 忽略车身附近点，前向检测起点 (m) |
| `max_forward` | 30.0 | 前方道路边界最大检测距离 (m) |
| `min_lateral` | 0.75 | 横向最小距离，过滤中央障碍物 (m) |
| `forward_bin_size` | 0.5 | 沿前进方向的采样间隔 (m) |
| `min_road_width` / `max_road_width` | 3.0 / 12.0 | 合法左右边界间距范围 (m) |
| `road_width_tolerance` | 1.5 | 相对于中位道路宽度的允许变化 (m) |
| `max_lateral_step` | 1.5 | 相邻纵向采样最大横向跳变 (m) |
| `min_points_per_side` | 2 | 单个纵向采样每侧最少障碍点数 |
| `smooth_window` | 5 | 车道线中值平滑窗口 |
| `cache_duration_ms` | 2000 | 世界坐标缓存时长 (ms) |

---

## 发布 / 订阅

| Topic | 方向 | 类型 |
|-------|------|------|
| `/patchworkpp/nonground` | 订阅 | `PointCloud2` |
| `/lidar/road_boundary_markers` | 发布 | `Marker` (LINE_STRIP ×2) |
| `/lidar/centerline` | 发布 | `Path` (可视化用) |

---

## 启动命令

```bash
cd /home/yaoh/lidar3d_ws && source install/setup.bash && ros2 launch lidar3d_bringup lidar_sim.launch.py
```

---

## 涉及文件

- `src/lidar3d_bringup/lidar3d_bringup/road_analyzer.py` — 全部改动集中在此文件
- `src/lidar3d_bringup/config/lidar_params.yaml` — 节点参数更新
- `src/lidar3d_bringup/test/test_road_analyzer.py` — 纵向双侧边界与缓存合并单元测试
