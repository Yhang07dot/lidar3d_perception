# lidar3d_ws — 3D LiDAR 感知工作区

智能巴哈赛车 16 线 3D 雷达数据处理与感知 pipeline。

## 快速启动

```bash
source /opt/ros/humble/setup.bash
source ~/lidar3d_ws/install/setup.bash
ros2 launch lidar3d_bringup play_and_viz.launch.py
```

## 项目结构

```
lidar3d_ws/
├── src/
│   ├── lidar3d_bringup/          # 启停 + TF + 过滤 + rviz 配置
│   │   ├── lidar3d_bringup/
│   │   │   ├── tf_publisher.py        # TF 发布 (odom→base_link + 静态外参)
│   │   │   └── pointcloud_filter.py   # 距离+高度过滤器
│   │   ├── launch/
│   │   │   └── play_and_viz.launch.py # 一键启动入口
│   │   └── rviz/
│   │       └── lidar3d.rviz           # rviz2 配置
│   └── patchwork-plusplus/       # 地面分割算法 (url-kaist)
├── bags -> ~/rosbag2_.../        # 数据包软链接
└── maps/                         # 预留：PCD 地图
```

## 数据流

```
rosbag → /cx/lslidar_point_cloud
  → pointcloud_filter (距离+高度)
  → Patchwork++ (地面分割: ground / nonground)
  → rviz2 三图层 (过滤点云 + 🟢地面 + 🔴障碍物)
```

## 关键参数

| 参数 | 位置 | 说明 |
|------|------|------|
| filter 距离/高度 | `pointcloud_filter.py` L28-31 | 默认值；launch 可覆盖 |
| sensor_height | launch `sensor_height:=1.5` | LiDAR 距地高度 |
| rviz2 显示效果 | rviz2 左侧 Displays 面板 | Decay/Size/Color 实时拖动 |

## 常用命令

```bash
# 自定义参数启动
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    sensor_height:=1.2 max_range:=20.0

# 关掉地面分割
ros2 launch lidar3d_bringup play_and_viz.launch.py enable_ground_seg:=false

# 运行时调 filter 参数
ros2 param set /pointcloud_filter max_range 30.0
```
