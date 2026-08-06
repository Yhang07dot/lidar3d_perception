# lidar3d_ws — 3D LiDAR 感知与聚类工作区

基于 ROS 2 Humble 的 3D LiDAR 感知 pipeline，支持 **rosbag 回放**、**Gazebo 仿真**、**实车 LiDAR** 三种数据源模式。

复杂感知计算、参数语义、坐标系和方案 A 的规划接口见
[`PERCEPTION_ALGORITHM.md`](PERCEPTION_ALGORITHM.md)。

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

### 当前仿真感知链（`lidar_sim.launch.py`）

```text
/lidar/points
  → pointcloud_filter → /cx/lslidar_point_cloud_filtered
  → Patchwork++ → /patchworkpp/nonground
  → surface_detector → /obstacles/boxes_3d_surface
  → obstacle_adapter → /obstacle_markers

/patchworkpp/nonground
  → road_analyzer → /road_boundary_markers、/lidar/centerline
```

- `obstacle_adapter` 将高置信 `obstacle_H...` 转为地图系静态 `tall` Track；Track 在
  空帧、短暂误分类和近场盲区中持续发布，车辆通过后才释放。
- `road_analyzer` 独立提取左右道路边界。当前帧可靠边界优先，地图缓存只补缺失 bin，
  从而避免避障姿态变化把历史直线段和当前边界混合成锯齿。
- 详细的坐标变换、置信度门槛、Track 生命周期和边界连续性规则见
  [`PERCEPTION_ALGORITHM.md`](PERCEPTION_ALGORITHM.md)。

### 2D 链路（默认，兼容旧版）

```text
输入点云 → pointcloud_filter → /cx/lslidar_point_cloud_filtered
  → Patchwork++ ──→ /patchworkpp/ground (地面)
                 └─→ /patchworkpp/nonground (非地面)
                       → euclidean_grid (2D 聚类) → /clusters/points
                         → cluster_bbox → /obstacles/boxes
                           → obstacle_adapter → /obstacle_markers
```

### 3D-PCA 链路（`use_3d_clustering:=true`）

```text
/patchworkpp/nonground
  → euclidean_cluster_3d (3D 体素聚类) → /clusters/points_3d
    → cluster_analyzer (PCA 特征 + 规则分类 + 时序追踪) → /obstacles/boxes_3d
      → obstacle_adapter → /obstacle_markers
```

### 3D-体素链路（`use_voxel_analyzer:=true`，2026-07-30 新增）

```text
/patchworkpp/nonground
  → voxel_analyzer (多分辨率体素网格 + 几何特征 + 体素聚类) → /obstacles/boxes_3d_voxel
    → obstacle_adapter → /obstacle_markers
```

体素网格：0-15m:0.1m, 15-30m:0.2m, 30-50m:0.4m。每格去最高 5% 浮点，提取 z_range/z_variance/density，26 邻域体素聚类。

**所有 3D 链路分类输出**：

| 类型 | type_id | 颜色 | 含义 |
|------|---------|------|------|
| **slope** | 3 | 深绿 | 可通过坡面 |
| **bump** | 2 | 黄 | 减速带/低坎 |
| **pole** | 1 | 红 | 杆状障碍物 |
| **obstacle** | 0 | 橙 | 不可通过 |

## 节点一览

| 可执行文件 | 节点名 | 功能 |
|-----------|--------|------|
| `tf_bridge` | `sensor_tf_bridge` | 动态 TF 广播 (base_link→sensor, 10Hz /tf) |
| `pointcloud_filter` | `pointcloud_filter` | 距离+高度+角度过滤 |
| `euclidean_cluster_3d` | `euclidean_cluster_3d` | 3D 体素洪水填充聚类 |
| `cluster_analyzer` | `cluster_analyzer` | **PCA 链路**: PCA特征+4类规则+时序追踪+置信度 |
| `voxel_analyzer` | `voxel_analyzer` | **体素链路**: 多分辨率网格+几何特征+体素聚类 |
| `cluster_bbox` | `cluster_bbox` | 2D 包围盒生成 |
| `obstacle_adapter` | `obstacle_adapter` | 语义适配+地图系静态 `tall` Track → `/obstacle_markers` |
| `road_analyzer` | `road_analyzer` | 左右道路边界稳定融合+中心线(调试) |
| `tf_publisher` | `tf_publisher` | rosbag 模式 TF 发布 |

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
| `rviz2_3d` | `lidar3d_3d.rviz` | 3D 聚类点云、PCA 分类标记（PCA 模式） |
| `rviz2_voxel` | `lidar3d_voxel.rviz` | 体素分类 (高/低置信)、地面/非地面（体素模式自动开启） |

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
