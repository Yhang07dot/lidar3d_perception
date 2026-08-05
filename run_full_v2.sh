#!/bin/bash
# 一体化启动: baja_cloud_sim-2.2 仿真 + LiDAR 感知
#
# 启动的节点总览:
#   仿真侧 (simulation.launch.py):
#     - gz sim (Gazebo server/client)
#     - ros_gz_bridge            (桥接 /lidar/points 等)
#     - robot_state_publisher
#     - truth_perception_node    (GPS/IMU/centerline 定位)
#     - frenet_planner_node      (消费 /obstacle_markers 避障)
#     - path_follower_node
#     - actuator_adapter_node
#     - evaluator_node
#     - video_recorder_node      (--record-video 时)
#     - rviz2                    (sim 的 1 个, --no-rviz 时关闭)
#   感知侧 (lidar_sim.launch.py):
#     - sensor_tf_bridge
#     - pointcloud_filter
#     - patchworkpp_node
#     - surface_detector_node    (C++)
#     - obstacle_adapter         (→ /obstacle_markers)
#     - rviz2 ×3                 (raw/processed/surface, --no-rviz 时关闭)
#
# 数据流:
#   Gazebo gpu_lidar → ros_gz_bridge → /lidar/points
#     → pointcloud_filter → patchworkpp → surface_detector → obstacle_adapter
#     → /obstacle_markers → frenet_planner_node (自主避障)

# 注意: 不使用 set -u / set -e, 因为 ROS 的 setup.bash 依赖未定义变量,
# 且脚本内大量条件赋值在 set -e 下会误杀进程。

SEED=42
OBSTACLES=5
HEADLESS_GAZEBO="true"
USE_VIDEO="false"
USE_RVIZ="true"

show_help() {
    cat << EOF
一体化启动脚本 (sim + LiDAR 感知)

用法: ./run_full_v2.sh [选项]

选项:
  --seed SEED              场景随机种子 (默认: 42)
  --obstacles N            障碍物数量 (默认: 5)
  --show-gazebo            显示 Gazebo GUI
  --record-video           录制视频
  --no-rviz                不显示任何 RViz (含 sim 的 1 个 + 感知的 3 个)
  --help, -h               显示帮助

示例:
  ./run_full_v2.sh
  ./run_full_v2.sh --seed 100 --obstacles 10
  ./run_full_v2.sh --show-gazebo
EOF
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --seed) SEED="$2"; shift 2 ;;
        --obstacles) OBSTACLES="$2"; shift 2 ;;
        --show-gazebo) HEADLESS_GAZEBO="false"; shift ;;
        --record-video) USE_VIDEO="true"; shift ;;
        --no-rviz) USE_RVIZ="false"; shift ;;
        -h|--help) show_help; exit 0 ;;
        *) echo "未知: $1"; show_help; exit 1 ;;
    esac
done

BAJA_SIM_PATH="/home/yaoh/baja_cloud_sim-2.2"
LIDAR_WS_PATH="/home/yaoh/lidar3d_ws"

if [ ! -d "$BAJA_SIM_PATH" ]; then
    echo "错误: 找不到 $BAJA_SIM_PATH"; exit 1
fi
if [ ! -d "$LIDAR_WS_PATH" ]; then
    echo "错误: 找不到 $LIDAR_WS_PATH"; exit 1
fi

# 确保 ROS 环境可用
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 重建 ros2 daemon, 避免残留的坏 daemon 导致节点发现异常 (ros2 topic list 永远空)
ros2 daemon stop >/dev/null 2>&1
sleep 1
ros2 daemon start >/dev/null 2>&1
sleep 1

# 确保 RViz 能找到显示器 (headless 服务器上无 DISPLAY 时回退到 :1)
if [ -z "${DISPLAY:-}" ]; then
    if [ "$USE_RVIZ" = "true" ]; then
        echo "警告: 当前无 DISPLAY 环境变量, RViz 可能无法弹出。尝试 DISPLAY=:1"
    fi
    export DISPLAY=":1"
fi

echo "========================================="
echo "  sim-2.2 + LiDAR 感知 一体化启动"
echo "========================================="
echo "  seed:       $SEED"
echo "  obstacles:  $OBSTACLES"
echo "  Gazebo GUI: $([ "$HEADLESS_GAZEBO" = "true" ] && echo "headless" || echo "显示")"
echo "  video:      $([ "$USE_VIDEO" = "true" ] && echo "是" || echo "否")"
echo "  RViz:       $([ "$USE_RVIZ" = "true" ] && echo "显示 (sim×1 + 感知×3)" || echo "隐藏")"
echo "  DISPLAY:    $DISPLAY"
echo "========================================="

# 构建仿真命令
cd "$BAJA_SIM_PATH"
SIM_CMD="./run.sh --seed $SEED --obstacles $OBSTACLES"
[ "$HEADLESS_GAZEBO" = "true" ] && SIM_CMD="$SIM_CMD --headless-gazebo"
[ "$USE_VIDEO" = "false" ] && SIM_CMD="$SIM_CMD --no-video"
[ "$USE_RVIZ" = "false" ] && SIM_CMD="$SIM_CMD --no-rviz"

# 用 setsid 启动仿真, 使其成为独立进程组长, 便于整体清理
# 通过写 $$ (组长 PID) 到文件获取准确的 PGID
echo ""
echo "[1/2] 启动仿真 (Gazebo + bridge + control)..."
rm -f /tmp/sim_pgid
setsid bash -c "echo \$\$ > /tmp/sim_pgid; cd '$BAJA_SIM_PATH' && exec $SIM_CMD" >/tmp/sim_full.log 2>&1 &
SIM_PGID=$(cat /tmp/sim_pgid 2>/dev/null || echo "")

# 等待 LiDAR 点云就绪 (比 /clock 更贴近"点云收到")
# 先等 /clock, 再等 /lidar/points, 确保整条链路发布者已起
echo "[2/2] 等待仿真初始化 (检测 /clock 与 /lidar/points)..."
MAX_WAIT=90
COUNTER=0
CLOCK_OK=0
LIDAR_OK=0
while [ $COUNTER -lt $MAX_WAIT ]; do
    if [ $CLOCK_OK -eq 0 ] && ros2 topic list 2>/dev/null | grep -q "/clock"; then
        CLOCK_OK=1
        echo "  ✓ /clock 就绪"
    fi
    if [ $LIDAR_OK -eq 0 ] && ros2 topic list 2>/dev/null | grep -q "/lidar/points"; then
        LIDAR_OK=1
        echo "  ✓ /lidar/points 存在"
    fi
    if [ $CLOCK_OK -eq 1 ] && [ $LIDAR_OK -eq 1 ]; then
        sleep 3
        break
    fi
    echo "  等待... (clock:$CLOCK_OK lidar:$LIDAR_OK, ${COUNTER}s/${MAX_WAIT}s)"
    sleep 3
    COUNTER=$((COUNTER + 3))
done

if [ $LIDAR_OK -eq 0 ]; then
    echo "警告: /lidar/points 在 ${MAX_WAIT}s 内未就绪, 感知链可能收不到点云。"
    echo "      检查 baja_cloud_sim 是否已 colcon build (install 内 bridge.yaml 含 lidar 段)。"
fi

# 启动 LiDAR 感知链 (独立进程组)
echo ""
echo "========================================="
echo "  启动 LiDAR 感知链 (filter→patchworkpp→detector→adapter)"
echo "========================================="
cd "$LIDAR_WS_PATH"
source install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

PERCEP_RVIZ_ARG=""
[ "$USE_RVIZ" = "false" ] && PERCEP_RVIZ_ARG="use_rviz:=false"
rm -f /tmp/percep_pgid
setsid bash -c "echo \$\$ > /tmp/percep_pgid; cd '$LIDAR_WS_PATH' && source install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; export DISPLAY=$DISPLAY; exec ros2 launch lidar3d_bringup lidar_sim.launch.py $PERCEP_RVIZ_ARG" >/tmp/percep_full.log 2>&1 &
PERCEP_PGID=$(cat /tmp/percep_pgid 2>/dev/null || echo "")

# 给感知链一点时间拉起节点
sleep 5
echo ""
echo "=== 当前节点清单 ==="
ros2 node list 2>/dev/null || echo "(节点列表获取失败, 可能为 daemon 问题)"
echo ""
echo "========================================="
echo "  已启动:"
echo "    仿真进程组 PGID=$SIM_PGID (Gazebo + bridge + control + sim rviz)"
echo "    感知进程组 PGID=$PERCEP_PGID (filter/patchworkpp/detector/adapter + 3×rviz)"
echo "  控制链: /lidar/points → perception → /obstacle_markers → frenet_planner"
echo "  日志:   仿真 /tmp/sim_full.log | 感知 /tmp/percep_full.log"
echo "  退出:   Ctrl+C 关闭全部"
echo "========================================="

# 捕获退出信号, 清理整棵进程树
cleanup() {
    echo ""
    echo "正在关闭 (kill 进程组)..."
    [ -n "$PERCEP_PGID" ] && kill -TERM -"$PERCEP_PGID" 2>/dev/null || true
    [ -n "$SIM_PGID" ] && kill -TERM -"$SIM_PGID" 2>/dev/null || true
    # 兜底: 清理可能残留的 gz sim 与 rviz
    pkill -TERM -f "gz sim" 2>/dev/null || true
    pkill -TERM -f "lidar_sim.launch" 2>/dev/null || true
    pkill -TERM -f "simulation.launch" 2>/dev/null || true
    pkill -TERM -f "rviz2" 2>/dev/null || true
    sleep 2
    echo "已退出"
    exit 0
}
trap cleanup EXIT INT TERM

wait
