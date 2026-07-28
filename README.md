# lidar3d_ws — 3D LiDAR 感知与聚类工作区

基于 ROS 2 Humble 的 3D LiDAR 感知 pipeline，支持 **rosbag 回放**、**Gazebo 仿真**、**实车 LiDAR** 三种数据源模式。pipeline 覆盖：TF 发布 → 点云过滤 → Patchwork++ 地面分割 → 欧几里得聚类 → 包围盒生成 → 障碍物分类与位姿变换。

## 三种启动模式

### rosbag 回放（默认）
```bash
ros2 launch lidar3d_bringup play_and_viz.launch.py
```

### Gazebo 仿真（配合 BajaSimPart）
```bash
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=simulation
```

### 实车 LiDAR
```bash
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=lidar
```

## 数据流

```text
[rosbag|Gazebo|LiDAR] → /lidar/points 或 /cx/lslidar_point_cloud
  → pointcloud_filter（距离+高度过滤）
  → Patchwork++（地面/非地面分割）
  → euclidean_grid（欧几里得聚类）
  → cluster_bbox（3D 包围盒）
  → obstacle_adapter（障碍物分类 + TF 变换 → /obstacle_markers）
  → RViz 双窗口可视化
```

## 主要包

| 包 | 说明 |
|----|------|
| `lidar3d_bringup` | 启动流程、TF 发布、点云过滤、bbox 生成、障碍物适配 |
| `lidar_cluster_ros2` | 点云聚类（Euclidean / DBSCAN / DBlane） |
| `patchwork-plusplus` | Patchwork++ 地面分割算法 |

## 第三方代码来源

| 目录 | 来源 | 分支/版本 | 改动 |
|------|------|-----------|------|
| `src/patchwork-plusplus` | https://github.com/url-kaist/patchwork-plusplus | v1.4.1 (`3e6903a`) | 无 |
| `src/lidar_cluster_ros2` | https://github.com/jkk-research/lidar_cluster_ros2 | ros2 (`17076fd`) | `euclidean_grid_core.hpp` 参数调整 |

## 依赖与环境

建议使用以下环境：

- ROS 2 Humble
- Python 3
- colcon
- PCL / pcl_ros
- numpy

如果是全新环境，建议先安装依赖：

```bash
cd ~/lidar3d_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 快速启动

### 1) 编译工作区

```bash
cd ~/lidar3d_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2) 启动完整可视化流程

```bash
ros2 launch lidar3d_bringup play_and_viz.launch.py
```

默认会使用工作区下的 bag 目录，并同时启动：

- rosbag 播放
- TF 发布
- 点云过滤
- Patchwork++ 地面分割
- 聚类与 bbox 生成
- 两个 RViz 窗口

### 3) 可选参数

```bash
# 自定义传感器高度和过滤距离
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    sensor_height:=1.2 max_range:=20.0

# 关闭地面分割模块
ros2 launch lidar3d_bringup play_and_viz.launch.py enable_ground_seg:=false
```

## 关键参数

| 参数 | 说明 |
|------|------|
| max_range / min_range | 控制点云过滤时的距离范围 |
| min_height / max_height | 控制点云过滤时的高度范围 |
| sensor_height | Patchwork++ 需要的传感器离地高度 |
| enable_ground_seg | 是否开启地面分割与后续聚类链路 |

## 常用调试命令

```bash
# 动态修改过滤参数
ros2 param set /pointcloud_filter max_range 30.0

# 查看当前节点
ros2 node list

# 查看话题
ros2 topic list

#You can just adjust the parameter in the rvizFile

拿到实车后还需要：

1. 用真实标定值替换当前估算的 TF 外参。
2. 为不同 bag 数据准备统一的配置文件和启动脚本。
3. 补充自动化测试与 launch test，减少回归风险。
4. 把当前的可视化结果和检测指标一起整理成更直观的评估流程。
5. 为项目增加更完整的 package metadata，替换现有的 TODO 描述。

## 未完成的下一步

- 把当前的点云处理链路整理成更清晰的“感知 pipeline”文档
- 增加目标跟踪与轨迹管理，让聚类结果更有连续性
- 结合地图构建或局部栅格地图，提升场景理解能力
- 把演示效果整理成视频或截图，便于分享和汇报

