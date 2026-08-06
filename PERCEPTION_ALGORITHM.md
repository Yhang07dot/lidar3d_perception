# LiDAR 感知算法与接口契约

本文档描述 `lidar3d_ws` 当前启用的感知计算、参数含义和与
`baja_cloud_sim-2.2` 控制链的接口约定。它对应“方案 A”：保留仿真提供的
`/reference_centerline` 与定位；LiDAR 感知负责道路两侧边界和障碍物。

## 1. 运行时数据流

```text
/lidar/points
  -> pointcloud_filter
  -> /cx/lslidar_point_cloud_filtered
  -> patchworkpp_node
       -> /patchworkpp/ground -> surface_detector
       -> /patchworkpp/nonground -> surface_detector
                                  -> road_analyzer
  -> surface_detector -> /obstacles/boxes_3d_surface
  -> obstacle_adapter -> /obstacle_markers

road_analyzer -> /road_boundary_markers

/reference_centerline + /road_boundary_markers + /obstacle_markers
  -> frenet_planner_node -> /planned_path
  -> path_follower_node -> /cmd_control
  -> actuator_adapter_node -> /model/baja_vehicle/cmd_vel
```

`lidar_sim.launch.py` 将 `road_analyzer` 的内部
`/lidar/road_boundary_markers` 重映射为 `/road_boundary_markers`。
规划器使用这个最终 topic；调试时应检查最终 topic，而不是内部名称。

## 2. 坐标系和消息约定

- LiDAR 输入帧通常是 `baja_vehicle/base_link/lidar`；`sensor_tf_bridge`
  发布它相对于 `base_link` 的静态变换。
- `base_link` 采用 ROS 车体约定：`x` 向前，`y` 向左，`z` 向上。
- 路沿 marker 的 `frame_id` 是 `base_link`，命名空间必须为 `road_left`
  和 `road_right`。`frenet_planner_node` 根据命名空间识别左右侧。
- `obstacle_adapter` 发布的 `tall` marker 参与横向避障；`flat_ground`
  marker 只影响纵向降速。

## 3. 点云裁剪：`pointcloud_filter`

实现：`src/lidar3d_bringup/lidar3d_bringup/pointcloud_filter.py`。

每个输入点 `(x, y, z)` 通过以下三个判据：

1. 三维距离：`min_range <= sqrt(x^2 + y^2 + z^2) <= max_range`；
2. 高度：`min_height <= z <= max_height`；
3. 水平视场：`abs(atan2(y, x)) <= angle_limit_deg`。

裁剪在保持原始 `PointCloud2` 字段布局的前提下复制保留点的字节段，避免
丢失 `ring`、intensity 等后续算法可能使用的字段。它的发布 topic 固定为
`/cx/lslidar_point_cloud_filtered`。

## 4. 地面建模与曲面残差：`surface_detector`

实现：`src/lidar3d_perception_cpp/include/lidar3d_perception_cpp/surface_detector.hpp`
和 `src/lidar3d_perception_cpp/src/surface_detector_node.cpp`。

### 4.1 极坐标地面栅格

Patchwork++ 的 ground 点按照极坐标 `(r, theta)` 入栅格。径向格宽为：

```text
dr(r) = dr_base + dr_per_m * r
```

因此远距离格更宽，用更少的点维持稳定估计。每格不直接采用普通平均高度，
而是从最低三分之一点中取中位数 `z_ground`；这使斜坡上被 Patchwork++ 错分到
ground 的高障碍物不容易抬高地面参考。离散度用
`MAD = median(abs(z - median(z)))` 估计。

超过 `z_ground + max(outlier_factor * MAD, 0.15 m)` 的 ground 点会作为错分
异常点并入后续 nonground 分析，而不会永久消失。

### 4.2 高度残差和自适应阈值

对每个待检点计算：

```text
residual = z_point - S(r, theta)
```

其中 `S` 是平滑后的地面曲面。阈值由近、远距离阈值随距离插值得到，避免用一
个固定高度差同时处理近场密集点和远场稀疏点。近场若曲面栅格样本不足，则用
全局地面参考替代插值曲面，防止 16 线雷达近场无 ground 回波时产生大量伪障碍。

超过阈值的点被聚类；每个簇使用主方向、垂直度、尺寸和高度等几何特征分类为
`obstacle`、`passable_low`、`passable_high`、`boundary` 或 `unknown`。置信度
综合点数、几何特征和时序历史，低于 `confidence_threshold` 的结果仅发往低
置信调试 topic。

## 5. 障碍物适配与时序留存：`obstacle_adapter`

实现：`src/lidar3d_bringup/lidar3d_bringup/obstacle_adapter.py`。

适配器将五类曲面分类压缩成规划器需要的两类 marker：

- 不可通过/边界类映射为 `tall`，供 Frenet 横向走廊避让；
- 可通过地形类映射为 `flat_ground`，供跟随器减速，不触发横向绕行。

直接缓存 `base_link` 坐标会让旧障碍物粘在车身前方。因此缓存写入时先做
`base_link -> map` 变换，发布时再做 `map -> base_link` 变换。车辆前进后，旧
障碍物在车体系中的 `x` 会自然减小。`obstacle_memory_ms` 控制遮挡容忍与幽灵
障碍保留时间的平衡。

## 6. 道路双侧边界：`road_analyzer`

实现：`src/lidar3d_bringup/lidar3d_bringup/road_analyzer.py`。

### 6.1 输入变换和前向裁剪

`/patchworkpp/nonground` 先转换到 `base_link`，随后仅保留：

```text
min_forward <= x <= max_forward
sqrt(x^2 + y^2) <= max_forward
```

这排除了车后点和车身近场点，避免 `LINE_STRIP` 沿 360 度点云绕回车身。

### 6.2 纵向 bin 的内侧障碍物面

前方道路按 `forward_bin_size` 沿 `x` 分段。每个 bin 中：

- 左侧候选为 `y >= min_lateral` 点的第 10 百分位；
- 右侧候选为 `y <= -min_lateral` 点的第 90 百分位。

该分位数近似道路内侧障碍物面：相比极值，它不会被单个离群回波拉向车道中央；
相比均值，它不会被障碍物外侧表面拉离道路。每侧至少需要
`min_points_per_side` 个点。

### 6.3 道宽一致性和稀疏 16 线处理

对同一纵向 bin 的左右候选计算：

```text
width = y_left - y_right
```

先要求 `min_road_width <= width <= max_road_width`，再以所有候选的中位道宽为
参考，丢弃偏差超过 `road_width_tolerance` 的配对。中位数保证少量车道内障碍或
远处外侧墙面不会改变主道路宽度。

16 线雷达在道路两侧常出现错位回波：相邻有效 pair 之间可能间隔数米。当前算法
保留这种宽度一致的稀疏支撑，只在相邻 0.5 m 采样发生大于
`max_lateral_step` 的横向突跳时剔除。随后按 `x` 排序连接 marker，而不再保留
“最长连续 bin 段”这一会清空稀疏边界的约束。

在 2026-08-06 的仿真帧中，可靠支撑位于约 `x = 4.7, 5.1, 9.7, 10.2, 14.7,
24.7 m`，左右位置分别约为 `+4.1 m` 和 `-4.1 m`，中位道路宽约 `8.2 m`。

### 6.4 平滑、缓存和中心线

每侧轨迹按 `x` 排序后使用滚动中位数平滑 `y`，既保留道路弯曲趋势，又能抑制单
帧回波尖刺。缓存去重同样按纵向 bin 完成；同一帧只写入一次 `map` 坐标缓存，
避免定时发布时不断重复同一帧点。

可视化中心线仅在左右线的共同 `x` 范围内插值：

```text
y_center(x) = (interpolate(y_left, x) + interpolate(y_right, x)) / 2
```

当前方案 A 的规划器仍使用真值 `/reference_centerline` 作全局参考；
`/lidar/centerline` 仅用于调试和未来替换真值参考线的接口准备。

## 7. 方案 A 的规划责任边界

`frenet_planner_node` 将 `road_left` 和 `road_right` marker 从车体系转换到世界
系，并为参考中心线的每个预瞄点选择最近的左右边界，收紧可行横向范围。
它只接受 `tall` 障碍物参与横向避障。故方案 A 的职责划分如下：

| 信息 | 当前来源 | 规划用途 |
|---|---|---|
| `/reference_centerline` | `truth_perception_node` | 全局行驶方向与 Frenet 参考 |
| `/gps/fix`、`/imu/yaw` | `truth_perception_node` | 仿真定位与航向 |
| `/road_boundary_markers` | `road_analyzer` | LiDAR 道路横向走廊约束 |
| `/obstacle_markers` | `obstacle_adapter` | LiDAR 障碍物避让/降速 |

因此，车辆是否可走和如何绕开障碍物由感知结果决定；全局赛道走向暂时仍由真值
中心线提供。要进入方案 B，需将感知中心线稳定变换到 `map` 并替代
`/reference_centerline`，同时引入非真值定位源。

## 8. 运行时验收

启动后应依次验证：

```bash
ros2 topic hz /lidar/points
ros2 topic hz /patchworkpp/nonground
ros2 topic echo --once /road_boundary_markers
ros2 topic info -v /obstacle_markers
ros2 topic echo --once /planner/status
ros2 topic info -v /cmd_control
```

预期 `/road_boundary_markers` 含 `road_left`、`road_right` 且 `points` 非空；
`/planner/status` 为 `FEASIBLE` 后，`/cmd_control` 应由 `path_follower_node`
持续发布。若规划可行而 `/cmd_control` 没有发布者，问题在控制节点生命周期或
控制输入，不在 LiDAR 边界检测。
