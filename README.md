# lidar3d_ws — 3D LiDAR 感知与聚类工作区

这是一个基于 ROS 2 Humble 的 3D LiDAR 感知工程，当前目标是把 rosbag 数据流跑通，并在 RViz 中直观观察点云处理效果。项目已经具备一个较完整的“感知 demo”雏形：从 rosbag 重播、TF 发布、点云过滤，到 Patchwork++ 地面分割、聚类和检测框可视化，都已经串起来了。

## 项目现状

项目现在已经具备了以下能力：

- 支持从 rosbag 回放 3D LiDAR 点云
- 发布 TF：从 odom 到 base_link，并提供传感器静态外参
- 对点云做距离和高度过滤，减少无效观测
- 接入 Patchwork++ 做地面/非地面分割
- 将非地面点云送入聚类节点，生成障碍物簇
- 通过 RViz 展示原始点云与处理后点云的对比效果

从工程角度看，这已经不是一个“空壳项目”了，而是一个能用来做算法调试、数据验证和视觉展示的工作区。

## 主要包

- [src/lidar3d_bringup](src/lidar3d_bringup)：负责启动流程、TF 发布、点云过滤、聚类结果可视化。
- [src/lidar_cluster_ros2](src/lidar_cluster_ros2)：提供点云聚类相关实现，当前 launch 里使用的是其中的 Euclidean 聚类节点。
- [src/patchwork-plusplus](src/patchwork-plusplus)：Patchwork++ 地面分割算法实现。

## 数据流

```text
rosbag / PointCloud2
  → 点云过滤（距离 + 高度）
  → Patchwork++ 地面分割
  → 非地面点云聚类
  → 检测框 / Marker 可视化
  → RViz 展示
```

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

这个项目已经能跑通一个很实用的 demo，但如果想继续往“工程化”推进，建议优先做这几件事：

1. 用真实标定值替换当前估算的 TF 外参。
2. 为不同 bag 数据准备统一的配置文件和启动脚本。
3. 补充自动化测试与 launch test，减少回归风险。
4. 把当前的可视化结果和检测指标一起整理成更直观的评估流程。
5. 为项目增加更完整的 package metadata，替换现有的 TODO 描述。

## 建议的下一步

如果你想把这个项目继续做深一点，可以从下面几个方向入手：

- 把当前的点云处理链路整理成更清晰的“感知 pipeline”文档
- 增加目标跟踪与轨迹管理，让聚类结果更有连续性
- 结合地图构建或局部栅格地图，提升场景理解能力
- 把演示效果整理成视频或截图，便于分享和汇报

