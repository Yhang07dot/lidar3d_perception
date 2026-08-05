#!/bin/bash
# 适配 baja_cloud_sim-2.2 的启动脚本
# 2.2 版本移除了 --use-lidar-perception 参数，需要手动启动感知流程

set -e

# 默认参数
SEED=42
OBSTACLES=5
HEADLESS_GAZEBO="true"
USE_VIDEO="false"
USE_RVIZ="true"
PERCEPTION_MODE="lidar"
BACKGROUND_SIM="false"

# 解析命令行参数
show_help() {
    cat << EOF
适配 baja_cloud_sim-2.2 的启动脚本

用法: ./run_full_v2.sh [选项]

选项:
  --seed SEED              场景随机种子 (默认: 42)
  --obstacles N            障碍物数量 (默认: 5)
  --show-gazebo           显示Gazebo GUI (默认: 隐藏)
  --record-video          录制视频
  --no-rviz               不显示RViz
  --perception-mode MODE  感知模式: lidar | truth | hybrid (默认: lidar)
  --background-sim        后台启动仿真（不会自动启动感知）
  --help, -h              显示此帮助信息

说明:
  由于 2.2 版本移除了 LiDAR 感知集成，本脚本会：
  1. 在后台启动仿真 (baja_cloud_sim-2.2/run.sh)
  2. 等待仿真初始化完成
  3. 启动 LiDAR 感知流程 (run_lidar.sh)

示例:
  # 默认启动 (LiDAR感知模式)
  ./run_full_v2.sh

  # 改变场景
  ./run_full_v2.sh --seed 100 --obstacles 10

  # 显示Gazebo GUI
  ./run_full_v2.sh --show-gazebo

  # 仅启动仿真（不启动感知，手动控制）
  ./run_full_v2.sh --background-sim
EOF
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --seed)
            SEED="$2"
            shift 2
            ;;
        --obstacles)
            OBSTACLES="$2"
            shift 2
            ;;
        --show-gazebo)
            HEADLESS_GAZEBO="false"
            shift
            ;;
        --record-video)
            USE_VIDEO="true"
            shift
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --perception-mode)
            PERCEPTION_MODE="$2"
            shift 2
            ;;
        --background-sim)
            BACKGROUND_SIM="true"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 检查路径
BAJA_SIM_PATH="/home/yaoh/baja_cloud_sim-2.2"
LIDAR_WS_PATH="/home/yaoh/lidar3d_ws"

if [ ! -d "$BAJA_SIM_PATH" ]; then
    echo "错误: 找不到 baja_cloud_sim-2.2"
    echo "路径: $BAJA_SIM_PATH"
    exit 1
fi

if [ ! -d "$LIDAR_WS_PATH" ]; then
    echo "错误: 找不到 lidar3d_ws"
    echo "路径: $LIDAR_WS_PATH"
    exit 1
fi

# 打印配置
echo "========================================="
echo "完整系统启动 (baja_cloud_sim-2.2)"
echo "========================================="
echo "仿真配置:"
echo "  场景种子:     $SEED"
echo "  障碍物数量:   $OBSTACLES"
echo "  Gazebo GUI:   $([ "$HEADLESS_GAZEBO" = "true" ] && echo "隐藏" || echo "显示")"
echo "  录制视频:     $([ "$USE_VIDEO" = "true" ] && echo "是" || echo "否")"
echo "  RViz:         $([ "$USE_RVIZ" = "true" ] && echo "显示" || echo "隐藏")"
echo ""
echo "感知配置:"
echo "  感知模式:     $PERCEPTION_MODE"
echo "  后台模式:     $([ "$BACKGROUND_SIM" = "true" ] && echo "是（不自动启动感知）" || echo "否")"
echo "========================================="

# 构建仿真命令
cd "$BAJA_SIM_PATH"

SIM_CMD="./run.sh --seed $SEED --obstacles $OBSTACLES"

if [ "$HEADLESS_GAZEBO" = "true" ]; then
    SIM_CMD="$SIM_CMD --headless-gazebo"
fi

if [ "$USE_VIDEO" = "false" ]; then
    SIM_CMD="$SIM_CMD --no-video"
fi

if [ "$USE_RVIZ" = "false" ]; then
    SIM_CMD="$SIM_CMD --no-rviz"
fi

echo ""
echo "步骤1: 启动仿真..."
echo "命令: $SIM_CMD"
echo ""

# 如果是后台模式，直接启动仿真并退出
if [ "$BACKGROUND_SIM" = "true" ]; then
    echo "后台模式: 仅启动仿真，不启动感知"
    echo "如需启动感知，请在新终端运行:"
    echo "  cd $LIDAR_WS_PATH"
    echo "  ./run_lidar.sh --perception-mode $PERCEPTION_MODE"
    exec $SIM_CMD
fi

# 前台模式：启动仿真并自动启动感知
# 创建一个临时脚本，在仿真启动后启动感知
TEMP_SCRIPT=$(mktemp)
cat > "$TEMP_SCRIPT" << 'EOFSCRIPT'
#!/bin/bash
# 等待仿真初始化
PERCEPTION_MODE="$1"
LIDAR_WS_PATH="$2"

echo ""
echo "========================================="
echo "步骤2: 等待仿真初始化..."
echo "========================================="

# 等待 /lidar/points 话题出现
MAX_WAIT=30
COUNTER=0
while [ $COUNTER -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/lidar/points"; then
        echo "✓ LiDAR 点云话题已就绪"
        break
    fi
    echo "等待 /lidar/points 话题... ($COUNTER/$MAX_WAIT)"
    sleep 1
    COUNTER=$((COUNTER + 1))
done

if [ $COUNTER -ge $MAX_WAIT ]; then
    echo "警告: 等待超时，但仍将启动感知流程"
fi

sleep 2  # 额外等待2秒确保所有节点就绪

echo ""
echo "========================================="
echo "步骤3: 启动 LiDAR 感知流程..."
echo "========================================="

cd "$LIDAR_WS_PATH"
exec ./run_lidar.sh --perception-mode "$PERCEPTION_MODE"
EOFSCRIPT

chmod +x "$TEMP_SCRIPT"

# 在后台启动感知流程
"$TEMP_SCRIPT" "$PERCEPTION_MODE" "$LIDAR_WS_PATH" &
PERCEPTION_PID=$!

# 启动仿真（前台）
exec $SIM_CMD
