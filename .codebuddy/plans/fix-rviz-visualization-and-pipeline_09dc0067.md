---
name: fix-rviz-visualization-and-pipeline
overview: 修复 rviz 可视化不显示障碍物/边界的根因（Fixed Frame odom 但缺少 odom→base_link TF），同时补齐缺失的 TF 桥接、清理 rviz 无用的 display、确保端到端数据流通。
todos:
  - id: fix-rviz-config
    content: 修改 lidar3d_surface_2d.rviz：Fixed Frame 从 odom 改为 base_link，禁用 Low-Confidence 和 Potholes 两个无数据源的 display
    status: completed
  - id: add-odom-tf
    content: 在 lidar_sim.launch.py 中新增 odom_map_bridge 静态 TF 节点（odom→map identity），参照 play_and_viz.launch.py 第115-120行的模式
    status: completed
  - id: rebuild-verify
    content: colcon build 并分两终端实测验证：/obstacle_markers、/road_boundary_markers、/patchworkpp/ground 在 rviz2 中均可见
    status: completed
    dependencies:
      - fix-rviz-config
      - add-odom-tf
---

## 用户需求

终端2启动感知链后，终端日志能正常打印障碍物识别结果（obstacle_adapter 每10帧输出 tall=N flat_ground=N），但 rviz2 的 2D 俯视窗口（lidar3d_surface_2d.rviz）不显示任何障碍物 Marker 或道路边界 LINE_STRIP，窗口一片空白。需要一次性修复可视化问题。

## 根因

rviz 配置文件 `lidar3d_surface_2d.rviz` 的 Global Options 中 `Fixed Frame` 设为 `odom`，但所有发布的数据（`/obstacle_markers`、`/road_boundary_markers`、`/patchworkpp/ground`）均在 `base_link` 或 `baja_vehicle/base_link/lidar` 坐标系下。当前 TF 树中只有 `map → base_link`（由 sim 侧 truth_perception_node 发布）和 `base_link → baja_vehicle/base_link/lidar`（由 sensor_tf_bridge 发布），缺少 `odom → base_link` 或 `odom → map` 的 TF 链路。rviz 无法将 base_link 坐标系的数据投影到 odom 坐标系，导致所有 Marker 和 PointCloud 不可见。

## 技术方案

### 修复策略

两处修改，一步到位：

1. **rviz 配置文件**：将 `Fixed Frame` 从 `odom` 改为 `base_link`，使 rviz 直接用 base_link 作为参考系，无需依赖缺失的 odom TF 链路。所有数据（`/obstacle_markers`、`/road_boundary_markers`、`/patchworkpp/ground`）都在 base_link 或其子帧下，rviz 可直接渲染。同时禁用无数据源的 display（`/lidar/low_confidence_surface`、`/lidar/pothole_markers`）避免 rviz 日志刷屏。

2. **launch 文件**：参照 `play_and_viz.launch.py` 第115-120行，在 `lidar_sim.launch.py` 中补充 `odom → map` 的 identity 静态 TF（`static_transform_publisher` 节点），确保 TF 树完整性以兼容任何依赖 odom 帧的节点。

### 改动文件

| 文件 | 改动 |
| --- | --- |
| `src/lidar3d_bringup/rviz/lidar3d_surface_2d.rviz` | Fixed Frame: odom → base_link；禁用 Low-Confidence 和 Potholes display |
| `src/lidar3d_bringup/launch/lidar_sim.launch.py` | 新增 odom_map_bridge 静态 TF 节点 |


### 不改动的文件

- `baja_cloud_sim-2.2` 侧任何文件不改
- `obstacle_adapter.py` 和 `road_analyzer.py` 不改（frame_id 已是 base_link，符合要求）

### 构建验证

- `colcon build --packages-select lidar3d_bringup`
- 分两次终端启动：终端1 `run.sh` + 终端2 `lidar_sim.launch.py`
- 验证 rviz2 窗口中 Obstacle Markers、Road Boundary、Ground 三个 display 均有内容渲染