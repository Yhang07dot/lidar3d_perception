#!/bin/bash
# 一体化启动: baja_cloud_sim-2.2 仿真 + LiDAR 感知
#
# 工作流:
#   1. 启动 sim-2.2 (Gazebo + bridge + control)
#   2. 检测 /lidar/points 就绪
#   3. 启动 lidar_sim.launch.py 感知链
#   4. obstacle_adapter → /obstacle_markers → frenet_planner 自动消费

set -e

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
  --no-rviz                不显示 RViz
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

echo "========================================="
echo "  sim-2.2 + LiDAR 感知 一体化启动"
echo "========================================="
echo "  seed:       $SEED"
echo "  obstacles:  $OBSTACLES"
echo "  Gazebo GUI: $([ "$HEADLESS_GAZEBO" = "true" ] && echo "headless" || echo "显示")"
echo "  video:      $([ "$USE_VIDEO" = "true" ] && echo "是" || echo "否")"
echo "  RViz:       $([ "$USE_RVIZ" = "true" ] && echo "显示" || echo "隐藏")"
echo "========================================="

# 构建仿真命令
cd "$BAJA_SIM_PATH"
SIM_CMD="./run.sh --seed $SEED --obstacles $OBSTACLES"
[ "$HEADLESS_GAZEBO" = "true" ] && SIM_CMD="$SIM_CMD --headless-gazebo"
[ "$USE_VIDEO" = "false" ] && SIM_CMD="$SIM_CMD --no-video"
[ "$USE_RVIZ" = "false" ] && SIM_CMD="$SIM_CMD --no-rviz"

# 启动仿真 (后台)
echo ""
echo "[1/2] 启动仿真..."
$SIM_CMD &
SIM_PID=$!

# 等待 ros2 bridge 就绪 (检测 /clock 话题)
echo "[2/2] 等待仿真初始化..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
MAX_WAIT=60
COUNTER=0
while [ $COUNTER -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/clock"; then
        echo "  ✓ /clock 就绪"
        sleep 3
        break
    fi
    echo "  等待... ($COUNTER/$MAX_WAIT)"
    sleep 2
    COUNTER=$((COUNTER + 2))
done

# 启动 LiDAR 感知 (新终端)
echo ""
echo "========================================="
echo "  启动 LiDAR 感知链"
echo "========================================="
cd "$LIDAR_WS_PATH"
source install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

ros2 launch lidar3d_bringup lidar_sim.launch.py &
PERCEP_PID=$!

echo ""
echo "========================================="
echo "  已启动: sim PID=$SIM_PID, 感知 PID=$PERCEP_PID"
echo "  控制链: /lidar/points → perception → /obstacle_markers → frenet_planner"
echo "========================================="

# 捕获退出信号
cleanup() {
    echo ""
    echo "正在关闭..."
    kill $PERCEP_PID 2>/dev/null || true
    kill $SIM_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "已退出"
}
trap cleanup EXIT INT TERM

wait
