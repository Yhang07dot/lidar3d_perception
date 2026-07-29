# lidar3d_ws — 3D LiDAR 感知与聚类工作区

基于 ROS 2 Humble 的 3D LiDAR 感知 pipeline，支持 **rosbag 回放**、**Gazebo 仿真**、**实车 LiDAR** 三种数据源模式。

## 启动命令速查

```bash
# ===== 基础模式 =====
ros2 launch lidar3d_bringup play_and_viz.launch.py                                    # rosbag 回放（默认）
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=simulation           # Gazebo 仿真
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=lidar                # 实车 LiDAR

# ===== 仿真 + 3D 聚类 + 坡面/障碍物区分（推荐仿真用）=====
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    input_source:=simulation use_3d_clustering:=true

# ===== 高级参数 =====
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    input_source:=simulation use_3d_clustering:=true \
    cloud_topic:=/my_points max_range:=25.0 sensor_height:=1.5

# 关闭地面分割（调试用）
ros2 launch lidar3d_bringup play_and_viz.launch.py enable_ground_seg:=false
```

## 数据流

### 2D 链路（默认，兼容旧版）

```text
输入点云 → pointcloud_filter → /cx/lslidar_point_cloud_filtered
  → Patchwork++ ──→ /patchworkpp/ground (地面)
                 └─→ /patchworkpp/nonground (非地面)
                       → euclidean_grid (2D 聚类) → /clusters/points
                         → cluster_bbox → /obstacles/boxes
                           → obstacle_adapter → /obstacle_markers
```

### 3D 链路（`use_3d_clustering:=true` 时启用，与 2D 并行）

```text
/patchworkpp/nonground
  → euclidean_cluster_3d (3D 体素聚类) → /clusters/points_3d
    → cluster_analyzer (PCA 几何分析 + 规则分类) → /obstacles/boxes_3d
      → obstacle_adapter → /obstacle_markers
```

**3D 链路分类输出**：

| 类型 | type_id | 颜色 | 判断条件 |
|------|---------|------|---------|
| **slope** (可通过坡面) | 3 | 深绿 | 平面性 >0.85, 倾角 <15°, 高度 <2m |
| **bump** (减速带/低坎) | 2 | 黄 | 低矮 (<0.25m) 或低平面 (<0.3m) |
| **pole** (杆状物) | 1 | 红 | 线性 >0.7, 高宽比 >2.5 |
| **obstacle** (不可通过) | 0 | 橙 | 其余非地面物体 |

## 全部启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_source` | `rosbag` | 数据源: `rosbag` / `simulation` / `lidar` |
| `use_3d_clustering` | `false` | 启用 3D 聚类 + PCA 分析（与 2D 并行） |
| `cloud_topic` | `__auto__` | 输入点云话题（自动解析：rosbag→`/cx/lslidar_point_cloud`, sim/lidar→`/lidar/points`） |
| `bag_dir` | `~/lidar3d_ws/bags` | [rosbag] rosbag2 目录 |
| `rate` | `1.0` | [rosbag] 回放速率 |
| `start_offset` | `0.0` | [rosbag] 跳过秒数 |
| `loop` | `true` | [rosbag] 循环播放 |
| `max_range` | `10.0` | 过滤最大距离 (m) |
| `min_range` | `0.1` | 过滤最小距离 (m) |
| `min_height` | `-3.0` | 过滤最低 Z (m) |
| `max_height` | `5.0` | 过滤最高 Z (m) |
| `sensor_height` | `1.5` | LiDAR 离地高度 (m) |
| `enable_ground_seg` | `true` | 启用地面分割 + 下游节点 |

## RViz 窗口

启动时自动打开 2~3 个窗口：

| 窗口名 | 配置文件 | 显示内容 |
|--------|---------|---------|
| `rviz2_raw` | `lidar3d_raw.rviz` | 过滤后原始点云 (`/cx/lslidar_point_cloud_filtered`) |
| `rviz2_proc` | `lidar3d_processed.rviz` | 地面/非地面、2D 包围盒 |
| `rviz2_3d` | `lidar3d_3d.rviz` | 3D 聚类点云、PCA 分类标记（仅 3D 模式） |

## 主要包

| 包 | 说明 |
|----|------|
| `lidar3d_bringup` | 启动流程、TF、点云过滤、聚类、分类、障碍物适配 |
| `lidar_cluster_ros2` | 2D 欧几里得聚类 (C++) |
| `patchwork-plusplus` | Patchwork++ 地面分割 (C++) |

## 第三方代码来源

| 目录 | 来源 | 分支/版本 | 改动 |
|------|------|-----------|------|
| `src/patchwork-plusplus` | https://github.com/url-kaist/patchwork-plusplus | v1.4.1 (`3e6903a`) | 无 |
| `src/lidar_cluster_ros2` | https://github.com/jkk-research/lidar_cluster_ros2 | ros2 (`17076fd`) | `euclidean_grid_core.hpp` 参数调整 |

## 开发日志

### 2026-07-29 (Session 2) — 3D 聚类 + 坡面-障碍物区分
- 新增 `euclidean_cluster_3d.py`：3D 体素聚类（洪水填充，纯 numpy）
- 新增 `cluster_analyzer.py`：PCA 特征提取 + 规则分类（slope/bump/pole/obstacle）
- 新增 `lidar3d_3d.rviz`：3D 可视化配置
- `obstacle_adapter.py`：新增 `input_topic` 参数，支持 2D/3D 链路切换
- `play_and_viz.launch.py`：新增 `use_3d_clustering` 启动参数

### 2026-07-29 (Session 1) — 仿真数据链路修复
- 新建 `tf_bridge.py`：动态 TF 广播（`/tf`, 10Hz），替代 `static_transform_publisher`
- `play_and_viz.launch.py`：`use_sim_time` 条件化, Patchwork++/adapter 帧名条件化
- `obstacle_adapter.py`：`source_frame`/`target_frame` ROS 参数化
- `pointcloud_filter.py`：BEST_EFFORT QoS 兼容 Gazebo bridge
- BajaSimPart：合并 `v1.4-real_lqr` + 修复 NumPy/SciPy 兼容崩溃

### 2026-07-27~28 (C14~C23) — Pipeline 搭建
- C14~C18：集成欧几里得聚类 + bbox + 障碍物适配
- C19~C20：多数据源切换 launch (rosbag/simulation/lidar)
- C21~C23：仿真 TF 链路 + 调试记录

## 依赖与环境

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+, numpy
- PCL / pcl_ros
- colcon

```bash
cd ~/lidar3d_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 常用调试

```bash
ros2 param set /pointcloud_filter max_range 30.0      # 动态调参
ros2 node list                                           # 查看节点
ros2 topic list                                          # 查看话题
ros2 topic hz /cx/lslidar_point_cloud_filtered           # 过滤输出频率
ros2 topic echo /obstacles/boxes_3d --once              # 3D 分类结果
ros2 run tf2_ros tf2_echo base_link baja_vehicle/base_link/lidar  # TF 验证
```
