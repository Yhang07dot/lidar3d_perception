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

超过阈值的点被聚类；每个簇使用垂直度、尺寸和高度等几何特征分类为
`obstacle`、`passable_low`、`passable_high` 或 `unknown`。道路边界不在此处分类，
统一由 `road_analyzer` 从 nonground 点云提取。置信度
综合点数、几何特征和时序历史，低于 `confidence_threshold` 的结果仅发往低
置信调试 topic。

### 4.3 从 ground 提取坡面 `flat_ground`

坡面不再从 residual 簇的整体标签推断：坡面与坡上的高障碍物在二维连通聚类中可能属于同一
簇，先将整个簇放行为可通过会漏掉真正的 `tall`。因此 `tall` 仍完全沿用 Layer 2 的
`nonground + ground_outlier` 残差路径；坡面则只使用 Patchwork++ 已经判为 `ground` 的点。

地面点以 `slope_grid_resolution_m` 建成笛卡尔高度栅格。每个格在
`slope_fit_radius_m` 邻域拟合局部平面 `z = ax + by + c`，仅用纵向导数 `a` 得到局部
坡度，避免横向路拱被当作纵向坡。通过 `slope_min_grade_deg`、平面 RMS 与连续栅格数量后，
相邻坡格合并成一个坡面区域。该判定使用传感器实际地面几何，不依赖仿真 `scenario.json`、
道路中心线或固定水平 FOV。

一个坡面区域以 source `Marker.CUBE` 发布：`pose` 是区域中心，`scale.x` 是控制端可直接
使用的车辆前向长度 `span_x`，`scale.y/z` 为横向范围和高度范围。`text` 的格式固定为：

```text
passable_slope apex_x=<m> apex_y=<m> apex_z=<m> span_x=<m>
grade_deg=<deg> cells=<count> c=1.00
```

source topic 中 apex 在点云坐标系；adapter 发布 `/obstacle_markers` 时将 apex 转到
`base_link`，并保留其余字段。最终 marker 的 `ns=flat_ground`，`pose`/`scale` 同样在
`base_link`；坡上存在 `tall` 时该地形元数据仍会同时发布，供控制端独立决定减速时机。
这里的 `pose` 是坡面区域中心，不是 apex；`span_x` 与最终 `scale.x` 相同。控制端若只
使用现有几何接口，应按 `pose.x ± scale.x / 2` 计算坡段；若需坡顶精确位置，再解析
`text` 中的 `apex_*`。

## 5. 障碍物适配与静态 `tall` Track：`obstacle_adapter`

实现：`src/lidar3d_bringup/lidar3d_bringup/obstacle_adapter.py`。

适配器将四类曲面分类压缩成规划器需要的两类 marker：

- `obstacle` 映射为 `tall`，供 Frenet 横向走廊避让；
- 可通过地形类与 `unknown` 映射为 `flat_ground`，供跟随器减速，不触发横向绕行。

`flat_ground` 只代表当前帧可通过地形，因此不做跨帧缓存。`tall` 则维护独立的
静态 Track：

```text
输入 CUBE marker
  -> sensor -> base_link -> map
  -> 最近邻关联已有 tall Track，或创建新 Track
  -> map -> 当前 base_link
  -> /obstacle_markers
```

Track 保存 `{track_id, world_xyz, scale_xyz, hit_count, last_observation}`。关联只使用
地图系 `x/y` 距离，不使用上游 `marker.id`，因为聚类编号会随帧变化。位置和尺寸的
指数更新为：

```text
track_value <- (1 - alpha) * track_value + alpha * observation
```

默认 `alpha = 0.30`，用于压低簇中心和尺寸的逐帧跳动，不扩大障碍物。

### 5.1 建立、更新和删除条件

`surface_detector` 在 `marker.text` 中输出组合置信度 `c`。该分数来自点数、垂直性、
边缘比例和短期分类历史；正式高置信 topic 的基础门槛为 `0.35`。adapter 在此之上
使用两级门槛：

- `c >= 0.80`：单帧立即建立 `tall` Track；
- `c >= 0.60`：仅更新已有关联 Track；
- 低于 `0.60`：不创建、不更新长期 Track；
- 同位置 `flat_ground` 不覆盖 `tall` Track。

空 `MarkerArray` 仍会触发 adapter 回调；只要 Track 未删除，它就会继续发布给规划器。
Track 不按时间过期。每帧把 Track 转入当前 `base_link`，只有当
`track_base_x < -track_release_behind_m`（默认 `-2.0 m`）时才删除，表示车辆已通过
障碍物并留出车尾安全距离。这样近场盲区、瞬时空帧和短暂 `flat_ground` 误分类不会让
规划器重新忽略已确认的静态障碍物。

### 5.2 感知道路边界的 Track 排除

`road_analyzer` 发布 `road_left` 和 `road_right` 两条 `LINE_STRIP`。adapter 将每条线
从其消息坐标系转换到 `map`，因此车辆运动不会让已收到的边界点粘在旧车体系。发布或更新
Track 时，边界会先投回当前 `base_link`，并在候选的前向位置 `x` 处插值：

```text
left_y(x)  = 左边界在 x 处的横向位置
right_y(x) = 右边界在 x 处的横向位置
```

若两侧边界都有效，只有满足 `right_y(x) < obstacle_y < left_y(x)` 的对象才属于车道内，
允许创建新 Track。落在左右边界外侧的候选视为路侧物，不创建 Track。若只得到单侧
边界，只排除该边界外侧、且距离不超过 `boundary_exclusion_distance_m`（默认 `1.0 m`）
的候选。道路边界只参与新建候选的过滤，不能删除已确认 Track；已确认 Track 仍只按
车辆通过条件释放。这样边界缺失、相交或跳变不会杀死真实车道内障碍物。

“车道内”只决定边界过滤是否放行，并不自动创建 Track。新建或更新 Track 仍要求上游
marker 的 namespace 为 `obstacle`、文本语义以 `obstacle_H` 开头，且满足相应的 `c`
置信度门槛。因此普通地面车道漆线不会进入 Track；`passable_*`、`unknown` 或 tracker
遗留的非 `obstacle_H` 文本也不会影响 Track。

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
帧回波尖刺。边界缓存保存于 `map`，并只用于填补当前帧的缺失 bin：

```text
若当前帧在 bin(x) 有可靠 live 点：输出 live 点
否则：                           输出 map cache 投回当前 base_link 的点
```

因此避障时车辆朝向变化不会把旧车体系的直线段与新观测中值混合；地图中真实直路投回
当前车体系后仍是一条直线（可能相对车辆倾斜）。真实弯道的 live 点在地图中沿同一条
连续弧线分布，所以会被保留并自然显示为弯曲车道线。

写入缓存前，live 左右线分别与同侧近期地图缓存计算最近点距离的中位数：

```text
jump_side = median(min_distance(live_world_point, cached_world_points))
```

只有左右两侧的 `jump_side` 都不超过 `world_continuity_threshold_m`（默认 `0.8 m`）时，
本帧 live 边界才会进入输出和缓存；任一侧明显跳变时，整对 live 线被拒绝，暂由已有
map cache 补洞。这保证弯道的连续曲率不会被冻结，同时抑制由稀疏点云、遮挡或避障姿态
造成的数米级错误折线。同一输入帧只写入一次缓存，避免定时发布时重复累积点。

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
| `/road_boundary_markers` | `road_analyzer`（目标） | LiDAR 道路横向走廊约束 |
| `/obstacle_markers` | `obstacle_adapter` | LiDAR 障碍物避让/降速 |

因此，车辆是否可走和如何绕开障碍物由感知结果决定；全局赛道走向暂时仍由真值
中心线提供。要进入方案 B，需将感知中心线稳定变换到 `map` 并替代
`/reference_centerline`，同时引入非真值定位源。

当前 Baja v2.2 有一项待控制组处理的路由冲突：`truth_perception_node` 仍向
`/road_boundary_markers` 发布边界真值，而本工作区在 `perception_mode=lidar` 下也将
`road_analyzer` 发布到同名 topic。二者会成为同一 topic 的两个发布者，planner 可能交替
使用真值和 LiDAR 边界。正式 LiDAR 联调前必须关闭/重映射真值边界发布；保留 truth 的
GPS、IMU、里程计和 `/reference_centerline` 不受此要求影响。

## 8. 运行时验收

### 8.1 规划路径—感知输入离线诊断

偶发的 `/planned_path` 与车辆实际位置偏差不在本工作区直接修改控制器。使用
`tools/analyze_planning_divergence_bag.py` 对齐 bag 中的 `/planned_path`、
`/ground_truth/odom`、`/obstacle_markers` 与 `/road_boundary_markers`，输出：

- 路径首点到车辆位置的锚点误差；
- 车辆到路径最近点误差；
- 相邻规划路径的最大几何跳变；
- 每个异常窗口同步的 tall/flat 数量和左右边界点数。

诊断结果用于判断异常是否伴随感知输入跳变；不改变 Baja planner/follower 行为。

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

## 9. 控制交接接口（当前正式版本）

本节定义控制端消费的最终接口。上游 `/obstacles/boxes_3d_surface` 是感知内部 source
topic，控制端不得直接订阅；正式输入为 adapter 输出的 `/obstacle_markers`。

### 9.1 `/obstacle_markers`

- 类型：`visualization_msgs/msg/MarkerArray`。
- 坐标系：每个 marker 的 `header.frame_id = base_link`，采用 `x` 前、`y` 左、`z` 上。
- 公共字段：`type = CUBE`、`action = ADD`、当前 orientation 为单位四元数、
  `lifetime = 0.2 s`。控制端必须按新消息刷新，不能把过期 marker 当作仍有效目标。

| `ns` | 生命周期 | `pose` / `scale` 含义 | 控制语义 |
|---|---|---|---|
| `tall` | map 系静态 Track，车辆通过后删除 | `pose` 为车体系中心，`scale.x/y/z` 为当前轴对齐碰撞尺寸 | 横向避障；`id` 在同一 Track 生命周期内稳定。 |
| `flat_ground` | 当前帧地形，不做跨帧 Track | `pose` 为区域中心，`scale.x` 为前向坡长 `span_x`，`scale.y/z` 为横向/高度范围 | 仅纵向减速；`id` 为瞬时数组编号，禁止用于追踪。 |

坡面 `flat_ground.text` 是控制组扩展字段，格式固定、字段单位均为米或度：

```text
passable_slope apex_x=<m> apex_y=<m> apex_z=<m> span_x=<m>
grade_deg=<deg> cells=<count> c=1.00
```

`apex_*` 已由 adapter 转到 `base_link`；`span_x` 必须与 `scale.x` 一致。顶点与区域中心
不同：上坡时 apex 常接近区域远端，山包时 apex 可位于区域中部。因此 apex 和总长不足以
严格反推起止边界；当前接口的确定坡段是
`[pose.x - scale.x / 2, pose.x + scale.x / 2]`。

### 9.2 当前 Baja 控制行为与责任边界

当前 `path_follower_node` 已消费两类 marker：

- `tall`：按 `pose`、`scale` 进入横向安全处理；
- `flat_ground`：选择最近的前向区域，按 `pose.x ± scale.x / 2` 和
  `obstacle_classes.flat_ground.approach_distance` 平滑降到
  `obstacle_classes.flat_ground.slow_speed`，不执行横向绕行。

截至 **2026 年 8 月 7 日**，Baja 控制代码尚未解析 `text` 中的 `apex_*`、`grade_deg`、
`cells`。控制组如需根据坡顶距离、坡度或置信度设计更精细速度曲线，需要实现该固定格式的
解析；解析缺失、格式未知或字段非法时，应安全回退为现有 `pose/scale.x` 几何降速逻辑。

### 9.3 `/road_boundary_markers`

该 topic 同为 `visualization_msgs/msg/MarkerArray`，包含 `ns=road_left` 和
`ns=road_right` 的 `LINE_STRIP`。消息坐标系为 `base_link`，`points` 是相对 marker
`pose` 的折线点。Frenet planner 使用它收紧横向走廊；`/lidar/centerline` 只作调试，
当前全局行驶方向仍来自 `/reference_centerline`。

### 9.4 Baja 控制侧已完成的感知接入

当前 Baja v2.2 的接口基础已经具备：Gazebo 车体包含 10 Hz `gpu_lidar`，其
`PointCloud2` 经 `ros_gz_bridge` 到 `/lidar/points`；`truth_perception_node` 已停止发布
`/obstacle_markers`，因此障碍物完全来自本工作区的 LiDAR 链。

`frenet_planner_node` 订阅 `/obstacle_markers` 与 `/road_boundary_markers`，仅 `tall`
进入横向走廊避障；`path_follower_node` 订阅同一障碍 topic，对 `flat_ground` 按
`pose.x ± scale.x / 2` 做纵向降速，并接收 planner 的 `/metrics/planned_clearance` 供安全
状态机使用。`mock_perception_node` 只是无 LiDAR 的接口测试工具，真实 LiDAR 联调时不得
同时运行。
