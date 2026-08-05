# LiDAR 3D 感知系统启动指南

## 🚀 快速启动

### 完整仿真 + LiDAR 感知（推荐）

```bash
cd /home/yaoh/baja_cloud_sim-2.1
./run.sh --seed 42 --obstacles 5 --headless-gazebo --use-lidar-perception
```

**说明**：
- 启动 Gazebo 仿真 + 车辆 + LiDAR 传感器
- 自动启动完整感知流程（地面分割 + 障碍物检测 + 路沿检测）
- LiDAR 感知输出直接替换真值，供控制端使用
- 包含一个 RViz 窗口（仿真总览）

**可选参数**：
```bash
# 显示 Gazebo GUI（调试用）
./run.sh --seed 42 --obstacles 5 --use-lidar-perception

# 不显示任何可视化窗口（纯后台运行）
./run.sh --seed 42 --obstacles 5 --headless-gazebo --no-video --no-rviz --use-lidar-perception

# 改变场景随机种子和障碍物数量
./run.sh --seed 100 --obstacles 10 --headless-gazebo --use-lidar-perception

# 使用真值感知（不用 LiDAR，对照测试用）
./run.sh --seed 42 --obstacles 5 --headless-gazebo
```

---

## 📊 perception_mode 参数说明

`perception_mode` 控制 LiDAR 感知与真值的话题路由关系：

### Mode 1: `lidar`（默认，推荐）
```bash
./run.sh --seed 42 --obstacles 5 --headless-gazebo --use-lidar-perception
# 内部等价于 perception_mode:=lidar
```

**数据流**：
- LiDAR 感知 → `/obstacle_markers`, `/road_boundary_markers`
- 真值系统 → `/truth/obstacle_markers`, `/truth/road_boundary_markers`
- **控制端订阅**: `/obstacle_markers`, `/road_boundary_markers` (使用 LiDAR 数据)

**用途**: 正常运行，控制端使用 LiDAR 感知

---

### Mode 2: `truth`（对照测试）
```bash
cd ~/lidar3d_ws
source install/setup.bash
ros2 launch lidar3d_bringup play_and_viz.launch.py \
  input_source:=simulation \
  use_surface_detector:=true \
  use_lidar_perception:=true \
  perception_mode:=truth \
  use_rviz_proc:=true
```

**数据流**：
- LiDAR 感知 → `/lidar/obstacle_markers`, `/lidar/road_boundary_markers`
- 真值系统 → `/obstacle_markers`, `/road_boundary_markers`
- **控制端订阅**: `/obstacle_markers`, `/road_boundary_markers` (使用真值)

**用途**: 对照测试，控制端使用真值，但 LiDAR 感知仍在后台运行到独立话题

---

### Mode 3: `hybrid`（调试对比）
```bash
cd ~/lidar3d_ws
source install/setup.bash
ros2 launch lidar3d_bringup play_and_viz.launch.py \
  input_source:=simulation \
  use_surface_detector:=true \
  use_lidar_perception:=true \
  perception_mode:=hybrid \
  use_rviz_proc:=true
```

**数据流**：
- LiDAR 感知 → `/lidar/obstacle_markers`, `/lidar/road_boundary_markers`
- 真值系统 → `/truth/obstacle_markers`, `/truth/road_boundary_markers`
- **控制端订阅**: 需要手动选择订阅哪个

**用途**: 在 RViz 中同时显示 LiDAR 和真值，进行精度对比

---

## 🎨 可视化窗口

### 窗口 1: 仿真总览（simulation.rviz）
**自动启动条件**: `./run.sh` 不带 `--no-rviz`

**显示内容**：
- 车辆模型 + TF 树
- 真值障碍物（橙色）
- 真值路沿（蓝色线）
- LiDAR 感知障碍物（红色=tall，绿色=flat_ground）
- LiDAR 感知路沿（LINE_STRIP）
- 规划路径

**Frame**: `odom`

---

### 窗口 2: LiDAR 2D 俯视图（lidar3d_surface_2d.rviz）
**手动启动**：
```bash
cd ~/lidar3d_ws
source install/setup.bash
ros2 launch lidar3d_bringup play_and_viz.launch.py \
  input_source:=simulation \
  use_surface_detector:=true \
  use_lidar_perception:=true \
  use_rviz_proc:=true \
  use_rviz_raw:=false
```

**显示内容**：
- 地面点云（绿色）
- 障碍物原始检测（5类：obstacle/passable_low/passable_high/boundary/unknown）
- 障碍物最终输出（2类：tall=红，flat_ground=绿）
- 路沿线（LINE_STRIP）
- 坑洼标记（如果有）

**Frame**: `odom`

---

## 📡 关键话题

### 输入
- `/lidar/points` - LiDAR 原始点云（16线 VLP-16，10Hz）
- `/localization/odom` - 带噪声的定位（来自 truth_perception_node）

### 输出（感知结果）
**perception_mode=lidar 时**：
- `/obstacle_markers` - 障碍物（2类，ns=tall/flat_ground）
- `/road_boundary_markers` - 路沿（LINE_STRIP，ns=road_left/road_right）

**perception_mode=truth 时**：
- `/lidar/obstacle_markers` - LiDAR 障碍物
- `/lidar/road_boundary_markers` - LiDAR 路沿

### 中间话题（调试用）
- `/patchworkpp/ground` - 地面点云
- `/patchworkpp/nonground` - 非地面点云
- `/obstacles/boxes_3d_surface` - 障碍物原始检测（5类）
- `/lidar/centerline` - 道路中心线（Path）
- `/lidar/pothole_markers` - 坑洼检测

### 真值话题
- `/truth/obstacle_markers` - 真值障碍物
- `/truth/road_boundary_markers` - 真值路沿
- `/reference_centerline` - 参考中心线

---

## 🔧 分步启动（调试用）

### 步骤 1: 仅启动仿真
```bash
cd /home/yaoh/baja_cloud_sim-2.1
source install/setup.bash
ros2 launch baja_cloud_sim simulation.launch.py \
  scenario:=loop \
  seed:=42 \
  obstacles:=5 \
  headless_gazebo:=true \
  use_video:=false \
  use_rviz:=false \
  use_lidar_perception:=false
```

### 步骤 2: 手动启动 LiDAR 感知
```bash
# 新终端
cd ~/lidar3d_ws
source install/setup.bash
ros2 launch lidar3d_bringup play_and_viz.launch.py \
  input_source:=simulation \
  use_surface_detector:=true \
  use_lidar_perception:=true \
  perception_mode:=lidar \
  use_rviz_proc:=true \
  use_rviz_raw:=false
```

---

## 🐛 调试命令

### 检查节点运行状态
```bash
ros2 node list | grep -E "surface_detector|obstacle_adapter|road_analyzer|patchwork"
```

**期望输出**：
```
/obstacle_adapter
/patchworkpp_node
/road_analyzer
/surface_detector
```

### 检查话题发布者
```bash
ros2 topic info /obstacle_markers
ros2 topic info /road_boundary_markers
```

**期望输出**（perception_mode=lidar 时）：
```
Publisher count: 1  (来自 obstacle_adapter)
Publisher count: 1  (来自 road_analyzer)
```

### 查看障碍物分类
```bash
ros2 topic echo /obstacle_markers --once | grep "ns:"
```

**期望输出**：
```
ns: tall
ns: flat_ground
```

### 查看路沿线
```bash
ros2 topic echo /road_boundary_markers --once | grep "ns:"
```

**期望输出**：
```
ns: road_left
ns: road_right
```

### 检查点云频率
```bash
ros2 topic hz /lidar/points
```

**期望输出**: `average rate: 10.0`

### 查看 TF 树
```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf，检查 baja_vehicle/base_link/lidar → base_link 变换
```

---

## ⚙️ 参数文件

**配置文件**: `~/lidar3d_ws/src/lidar3d_bringup/config/lidar_params.yaml`

### 关键参数

#### PatchWork++ (地面分割)
```yaml
patchworkpp:
  ros__parameters:
    sensor_height: 1.5          # LiDAR 高度（m）
    verbose: false
```

#### surface_detector (障碍物检测)
```yaml
surface_detector:
  ros__parameters:
    min_cluster_size: 10        # 最小聚类点数
    max_cluster_size: 5000      # 最大聚类点数
    cluster_tolerance: 0.5      # 聚类距离阈值（m）
    boundary_height_threshold: 0.35  # 路沿高度阈值（m）
```

#### road_analyzer (路沿检测)
```yaml
road_analyzer:
  ros__parameters:
    angular_resolution: 1.0     # 极坐标角度分辨率（度）
    gap_threshold: 0.8          # 间隙检测阈值（m）
    smoothing_window: 5         # 平滑窗口大小
```

#### obstacle_adapter (障碍物分类映射)
```yaml
obstacle_adapter:
  ros__parameters:
    memory_duration_ms: 500     # 障碍物缓存时间（ms）
    dead_reckoning: true        # 启用推算定位
```

---

## 📝 常见问题

### Q1: surface_2d 窗口看不到障碍物
**A**: 检查 perception_mode 参数和话题订阅：
```bash
# 确认 obstacle_adapter 在发布
ros2 topic info /obstacle_markers

# 如果 Publisher count: 0，检查节点是否启动
ros2 node list | grep obstacle_adapter
```

### Q2: 路沿线不准确
**A**: 这是已知问题。road_analyzer 的 gap detection 算法需要调参，或者等仿真侧加入更明显的路沿几何。

### Q3: 仿真启动后卡住
**A**: 检查 Gazebo 是否正常启动：
```bash
ps aux | grep "gz sim"
# 如果没有输出，检查 /tmp/road_debug.log 或 ~/.gazebo/ 日志
```

### Q4: LiDAR 点云没有数据
**A**: 检查 ros_gz_bridge 是否正确桥接：
```bash
ros2 topic hz /lidar/points
# 如果没有输出，检查 bridge.yaml 中的话题映射
```

---

## 🔄 版本历史

- **2026-08-05**: 清理冗余节点 + 添加 perception_mode 参数
- **2026-08-04**: C++ surface_detector 成为默认 + BajaSimPart 集成
- **2026-08-03**: 颜色统一 + 置信度显示
- **2026-07-31**: surface_detector 性能优化 + YAML 配置化
- **2026-07-29**: 初始 LiDAR 感知流程

---

**维护者**: yaoh  
**最后更新**: 2026-08-05
