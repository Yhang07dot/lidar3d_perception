# lidar3d_ws — 当前 LiDAR 感知与控制接口

基于 ROS 2 Humble 的 LiDAR 感知工作区。当前正式交付目标是 Baja 仿真中的方案 A：
感知提供道路边界、静态 `tall` 障碍物和坡面 `flat_ground`；控制端负责路径规划、横向避障
与纵向减速。

复杂计算、参数语义、坐标系和验收边界见
[`PERCEPTION_ALGORITHM.md`](PERCEPTION_ALGORITHM.md)。

## 正式启动

```bash
# Terminal 1: 启动 Baja 仿真
cd ~/baja_cloud_sim-2.2
./run.sh

# Terminal 2: 启动当前正式感知链
cd ~/lidar3d_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lidar3d_bringup lidar_sim.launch.py
```

`play_and_viz.launch.py` 仅保留给 rosbag/旧链路调试。向控制组交付或进行当前仿真测试时，
使用 `lidar_sim.launch.py`，不要同时启动两套 launch。

## 当前正式数据流

```text
/lidar/points
  → pointcloud_filter → /cx/lslidar_point_cloud_filtered
  → patchworkpp_node ──→ /patchworkpp/ground
                       │     → surface_detector（连续坡面）
                       └─→ /patchworkpp/nonground
                             → surface_detector（残差 tall）
  → surface_detector → /obstacles/boxes_3d_surface
  → obstacle_adapter → /obstacle_markers

/patchworkpp/nonground
  → road_analyzer → /road_boundary_markers、/lidar/centerline（调试）
```

- `surface_detector_node` 的 `ground` 路径识别连续纵向坡面；`nonground` 残差路径识别
  `tall`，两者相互独立。
- `obstacle_adapter` 将高置信 `obstacle_H...` 转为 map 系静态 `tall` Track；空帧、短暂
  误分类和近场盲区不会删除已确认 Track，车辆通过后才释放。
- `road_analyzer` 的当前可靠边界优先，map 空间缓存只补缺失 bin；规划器据此收紧横向走廊。

## 节点状态：正式链路与不要启动项

| 节点 | 当前状态 | 原因 |
|------|----------|------|
| `tf_bridge` | 仿真必需 | 提供 `base_link → baja_vehicle/base_link/lidar`，供 adapter 与 RViz 使用。 |
| `pointcloud_filter` | 必需 | 统一输入范围、高度和水平视场。 |
| `patchworkpp_node` | 必需 | 提供 `ground`（坡面）和 `nonground`（障碍物/边界）两路输入。 |
| `surface_detector_node` | 必需 | C++ 主算法：残差 tall、ground 坡面和 source marker。 |
| `obstacle_adapter` | 必需 | 生成控制组订阅的 `/obstacle_markers`，维护 tall Track。 |
| `road_analyzer` | 必需 | 发布道路边界；adapter 用它过滤路侧误建 Track，规划器用它收紧横向走廊。 |
| `rviz2_surface` | 可选 | 仅可视化，`use_rviz:=false` 时不启动。 |

以下节点或旧链路**不属于当前正式仿真链路，不要与 `lidar_sim.launch.py` 同时启动**：

| 节点/链路 | 当前处理方式 | 原因 |
|-----------|--------------|------|
| `cluster_bbox` | 调试兼容保留 | 等待旧 `/clusters/points`；当前没有正式上游聚类器，不参与控制交付。 |
| `euclidean_grid`、`euclidean_cluster_3d`、`cluster_analyzer` | 已移除 | 已被 C++ `surface_detector_node` 替代。 |
| `voxel_analyzer`、`boundary_detector`、Python `surface_detector.py` | 已失效 | 当前无源码/console entrypoint；`lidar_params.yaml` 中对应块只保留历史参数参考。 |
| `tf_publisher` | rosbag 专用 | 为 `/chcnav/odom` 和 `laser_link` 构建 TF；标准仿真使用 `tf_bridge`，不需要它。 |
| `play_and_viz.launch.py` | 兼容/诊断 | 会按旧参数尝试启动备用路径；不作为当前控制组集成入口。 |

## 给控制组的正式接口

### `/obstacle_markers`

- **类型**：`visualization_msgs/msg/MarkerArray`。控制端应订阅此 topic，而非 source
  `/obstacles/boxes_3d_surface`。
- **坐标系**：每个 marker 的 `header.frame_id = base_link`，`x` 向前、`y` 向左、`z` 向上。
- **形状**：`type = CUBE`、`action = ADD`、当前 `pose.orientation` 为单位四元数，
  `scale.x/y/z` 为车体系轴对齐尺寸。`lifetime = 0.2 s`，控制端应按持续新消息刷新。

| `ns` | 语义 | 控制端可依赖字段 | 当前行为 |
|------|------|----------------|----------|
| `tall` | 不可通行静态障碍物 | `pose` 为中心；`scale.x/y/z` 为碰撞尺寸；`id` 为 Track 生命周期内稳定 ID | 参与 Frenet 横向避障；Track 在近场盲区/短暂漏检中继续发布，车辆通过后删除。 |
| `flat_ground` | 可通过特殊地形（当前为坡面） | `pose` 为区域中心；`scale.x = span_x` 为前向坡长；`scale.y/z` 为横向/高度范围 | 不参与横向避障；供纵向减速。flat marker 的 `id` 只在当前帧有效，不能用于追踪。 |

坡面 `flat_ground.text` 的固定格式如下，所有坐标已经是 `base_link`：

```text
passable_slope apex_x=<m> apex_y=<m> apex_z=<m> span_x=<m>
grade_deg=<deg> cells=<count> c=1.00
```

- `apex_*`：坡面最高地面栅格。
- `span_x`：与 `scale.x` 相同的前向坡长。
- `pose`：坡面区域中心，不是 apex。仅凭 apex 和总长度不能严格恢复起止边界；当前 Baja
  跟随器按 `pose.x ± scale.x / 2` 计算坡段。

**当前 Baja 控制代码的实际消费范围**：`path_follower_node` 已对 `flat_ground` 按
`pose.x` 和 `scale.x` 做渐进降速（不做横向绕行），也会对 `tall` 做横向安全处理；它**尚未
解析** `text` 中的 `apex_*`、`grade_deg` 与 `cells`。控制组若要按坡顶位置或坡度制定更精细
速度曲线，需要自行解析上述固定格式；解析失败时应回退到 `pose/scale.x` 几何逻辑。

### `/road_boundary_markers`

- **类型**：`visualization_msgs/msg/MarkerArray`，包含 `ns=road_left` 与 `ns=road_right`
  的 `LINE_STRIP`。
- **坐标系**：当前发布在 `base_link`；`points` 是相对 marker `pose` 的折线点。
- **用途**：规划器用其收紧横向可行走廊；`/lidar/centerline` 仅作调试，当前全局参考仍是
  `/reference_centerline`，控制端不要将其当作正式全局导航输入。

## 常用验证与调试

```bash
# 必需输入/输出速率和语义快照
ros2 topic hz /lidar/points
ros2 topic hz /patchworkpp/nonground
ros2 topic echo --once /obstacle_markers
ros2 topic echo --once /road_boundary_markers

# 检查仿真 sensor TF
ros2 run tf2_ros tf2_echo base_link baja_vehicle/base_link/lidar
```

检查坡面时，预期 `/obstacle_markers` 出现 `ns: flat_ground`，其 `text` 以
`passable_slope apex_x=` 开头；检查障碍物时，预期 `ns: tall` 持续发布至车辆通过。

### 规划路径偏差离线诊断

当仿真出现“规划路径与实际车辆偏差过大”时，先录制包含 `/planned_path`、
`/ground_truth/odom`、`/obstacle_markers`、`/road_boundary_markers` 的 bag，再运行：

```bash
source /opt/ros/humble/setup.bash
python3 tools/analyze_planning_divergence_bag.py \
  ~/rosbags/<bag目录> \
  --output /tmp/planning_divergence.md \
  --json-output /tmp/planning_divergence.json
```

该工具只定位路径锚点误差、路径跳变与同步感知输入；不修改 Baja planner/follower。

## 构建与依赖

```bash
cd ~/lidar3d_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

- ROS 2 Humble（Ubuntu 22.04）
- Python 3.10+、NumPy
- PCL / pcl_ros、Patchwork++、Eigen3
- `colcon`
