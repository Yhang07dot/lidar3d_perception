# 3D LiDAR 连接 Gazebo 仿真 — Session 总结

**日期**: 2026-07-28 ~ 2026-07-29  
**目标**: 将 lidar3d_ws 的感知 pipeline 对接 BajaSimPart 的 Gazebo 仿真  
**状态**: LiDAR 传感器已加入仿真模型并产生数据，但感知 pipeline 尚未成功消费仿真数据  

---

## 1. 背景

两个 ROS2 Humble workspace：

```
~/lidar3d_ws/       — 感知 pipeline (filter → Patchwork++ → clustering → bbox → classification)
~/BajaSimPart/      — 仿真 (Gazebo Fortress 6.18) + 规划控制 (origin: PandaFixLe/baja_cloud_sim)
```

规控组分支：`origin/v1.3-param.yaml`（有 camera sensor + wheel_odom）  
我们的分支：`lidar_perception`（在 v1.3 基础上加 LiDAR）

---

## 2. BajaSimPart 改动

### 2.1 文件改动

| 文件 | 改动 | 状态 |
|------|------|------|
| `models/baja_vehicle/model.sdf` | +`<sensor type="gpu_lidar">` 16线 LiDAR | ✅ |
| `config/bridge.yaml` | +`/lidar/points` 桥接条目 | ✅ |

### 2.2 LiDAR 传感器参数

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0.5 0 1.5 0 0 0</pose>           <!-- base_link 前方0.5m, 高1.5m -->
  <topic>/lidar/points</topic>
  <update_rate>10</update_rate>
  <always_on>true</always_on>              <!-- 必需！否则传感器不激活 -->
  <visualize>true</visualize>              <!-- Gazebo 中可视化射线 -->
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>            <!-- 每线点数 -->
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>     <!-- 360度 -->
      </horizontal>
      <vertical>
        <samples>16</samples>              <!-- 16线 -->
        <min_angle>-0.2618</min_angle>     <!-- -15° -->
        <max_angle>0.2618</max_angle>      <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>80</max>
    </range>
  </ray>
</sensor>
```

### 2.3 Bridge 配置

```yaml
- ros_topic_name: "/lidar/points"
  gz_topic_name: "/lidar/points/points"   # ← 注意：Gazebo 自动加 /points 后缀
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "ignition.msgs.PointCloudPacked"  # Fortress 用 ignition 命名空间
  direction: GZ_TO_ROS
```

### 2.4 编译注意事项

- `setup.py` 的 data_files 模式是 COPY（不是 symlink），改 SDF 后需要清 `build/` + `install/` 再重编
- 重新 clone 后变成 symlink 模式（`colcon build --symlink-install`），此后改 SDF 即时生效

---

## 3. lidar3d_ws 改动

### 3.1 文件改动

| 文件 | 改动 | 状态 |
|------|------|------|
| `play_and_viz.launch.py` | +`input_source` 参数 (rosbag/simulation/lidar) | ✅ |
| `play_and_viz.launch.py` | +`cloud_topic` 自动解析 (`__auto__` sentinel) | ⚠️ 未验证 |
| `play_and_viz.launch.py` | +static TF 桥接 (sim 模式: `odom→map`, `sensor→laser_link`) | ⚠️ 未验证 |
| `pointcloud_filter.py` | 话题改为 `input_cloud` (可 remap) | ✅ |
| `obstacle_adapter.py` | **新增** — 分类 (pole/bump/generic) + TF变换 + `/obstacle_markers` | ✅ |
| `setup.py` | 注册 obstacle_adapter 入口 | ✅ |

### 3.2 启动命令

```bash
# rosbag 模式（已验证可用）
ros2 launch lidar3d_bringup play_and_viz.launch.py

# 仿真模式（未验证成功）
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=simulation

# 仿真模式 + 自定义话题
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    input_source:=simulation cloud_topic:=/my_points
```

### 3.3 仿真模式的 TF 设计

```
rosbag 模式:
  我们发: odom → base_link (tf_publisher, 动态)
  我们发: base_link → laser_link (tf_publisher, 静态)

仿真模式:
  仿真发: map → base_link (AckermannSteering 插件)
  我们发: baja_vehicle/base_link/lidar → laser_link (static, identity)
  我们发: odom → map (static, identity)
  → rviz2 Fixed Frame = odom 可正常工作
```

---

## 4. 已验证的事实

### 4.1 正常工作的

| 项目 | 验证方式 | 结果 |
|------|---------|------|
| Gazebo LiDAR 内部话题 | `ign topic -l \| grep lidar` | `/lidar/points` + `/lidar/points/points` ✅ |
| ROS2 topic 存在 | `ros2 topic list \| grep lidar` | `/lidar/points` ✅ |
| ROS2 topic 有 publisher | `ros2 topic info /lidar/points` | Publisher count: 1 ✅ |
| LiDAR 点云有数据 | `ros2 topic echo /lidar/points --once --field header` (需等8秒) | frame_id: `baja_vehicle/base_link/lidar` ✅ |
| 点云字段 | 同上 | width=1800 (每线点数) ✅ |
| 仿真传感器已激活 | Gazebo 控制台无报错 | 正常 ✅ |
| rosbag 模式 pipeline | 所有节点正常 | filter→Patchwork→cluster→bbox→adapter ✅ |

### 4.2 不工作的

| 问题 | 现象 | 可能原因 |
|------|------|---------|
| **filter 未收到仿真数据** | `ros2 topic hz /cx/lslidar_point_cloud_filtered` 无输出 | remap 未生效或 cloud_topic 解析错误 |
| **rviz2 空白** | 仿真模式下两个窗口均无点云显示 | TF 链不完整 |
| **时间倒流 warning** | `Detected jump back in time` 持续刷屏 | Gazebo Fortress clock 不稳定，`use_sim_time` 配置冲突 |
| **仿真车辆卡顿/抖动** | 车运行不顺畅 | 未知（可能与 clock 或 LiDAR sensor 计算开销有关） |

---

## 5. 调试过程与试错记录

### 5.1 LiDAR sensor 不出现

| 尝试 | 结果 |
|------|------|
| 最初用 `gpu_lidar` + `topic:lidar` | 话题不存在 |
| 去掉 `always_on` 和 `visualize` 时 | 相机有这两项才工作 → 加上后仍无效 |
| 发现 `install/` 的 SDF 没有 LiDAR | `setup.py` data_files 是 copy 模式，增量编译不更新 → 清 build/install 重编 |
| `gz topic -l` 无输出 | 没设 `GZ_PARTITION` 环境变量 → 用 `ign topic -l` 代替 |
| bridge 订阅 `/lidar/points` | Gazebo 实际发在 `/lidar/points/points` → 修正 bridge YAML |
| bridge 用 `gz.msgs.PointCloudPacked` | Fortress 用 `ignition.msgs.PointCloudPacked` → 修正 |
| 尝试 `<frame_name>laser_link</frame_name>` | Fortress 6.18 不支持 → 撤回，改用 static TF |

### 5.2 感知 pipeline 对接

| 尝试 | 结果 |
|------|------|
| `PythonExpression` 做 cloud_topic 默认值 | `OpaqueFunction` 中 perform() 可能未正确 resolve → 改用 `__auto__` sentinel |
| 仿真模式未启动 | 终端残留多份仿真进程冲突 → pkill 清理 |
| 静态 TF `baja_vehicle/base_link/lidar → laser_link` | Identity 变换，未验证是否正确 |
| 静态 TF `odom → map` | 修复 rviz2 Fixed Frame 不匹配 |

---

## 6. 未解决问题

1. **filter 节点在仿真模式下未接收到 `/lidar/points` 数据** — `ros2 topic info /lidar/points` 显示 Subscription count: 0，说明 `input_cloud:=/lidar/points` remap 未生效
2. **时间倒流 (clock jump)** — 仿真侧 `robot_state_publisher` 持续 warning，感知侧 rviz2 也报。根源在 Gazebo Fortress `/clock` 不稳定
3. **仿真车辆运行卡顿** — LiDAR sensor 可能增加了 Gazebo 计算负载（gpu_lidar 需要 OGRE2 渲染）
4. **BajaSimPart 未设置 `.gitignore`** — `build/`, `install/`, `log/`, `results/` 都进了 git
5. **lidar_cluster_ros2 是裸 git clone** — 应改为 submodule（与 patchwork-plusplus 同类问题）

---

## 7. 建议的下一步

1. **先确认仿真侧 LiDAR 稳定** — 关闭 LiDAR sensor（注释掉），看车辆是否恢复顺畅。确认 LiDAR 增加了多少计算开销
2. **修复 filter 订阅** — 单独运行 `ros2 run lidar3d_bringup pointcloud_filter --ros-args -r input_cloud:=/lidar/points -p use_sim_time:=true`，确认能收到数据
3. **简化测试** — 不要一上来就跑完整 pipeline，先确认 filter 输出不为空，再加 Patchwork++
4. **clock 问题** — 检查是否有多份 `/clock` publisher（Gazebo + rosbag 同时发）。仿真模式应确保只有 Gazebo 发 `/clock`
5. **BajaSimPart 加 .gitignore** — 排除 build/install/log/results
