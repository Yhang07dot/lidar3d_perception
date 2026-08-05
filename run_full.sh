#!/bin/bash
# 完整系统启动脚本：仿真 + LiDAR感知 + 控制
# 用法: ./run_full.sh [选项]

set -e

# 默认参数
SEED=42
OBSTACLES=5
HEADLESS_GAZEBO="true"
USE_VIDEO="false"
USE_RVIZ="true"
USE_LIDAR_PERCEPTION="true"
PERCEPTION_MODE="lidar"
SHOW_LIDAR_2D="false"

# 解析命令行参数
show_help() {
    cat << EOF
完整系统启动脚本：仿真 + LiDAR感知 + 控制

用法: ./run_full.sh [选项]

选项:
  --seed SEED              场景随机种子 (默认: 42)
  --obstacles N            障碍物数量 (默认: 5)
  --show-gazebo           显示Gazebo GUI (默认: 隐藏)
  --record-video          录制视频
  --no-rviz               不显示RViz
  --show-lidar-2d         显示LiDAR 2D俯视图窗口
  --perception-mode MODE  感知模式: lidar | truth | hybrid (默认: lidar)
  --use-truth             使用真值感知 (等价于 --perception-mode truth)
  --help, -h              显示此帮助信息

感知模式说明:
  lidar   - LiDAR感知替换真值，控制端使用LiDAR识别的障碍物和路沿
  truth   - 控制端使用真值，LiDAR感知在后台运行（对照测试用）
  hybrid  - 两者并行，可在RViz中同时查看（调试对比用）

示例:
  # 默认启动 (seed=42, 5个障碍物, LiDAR感知)
  ./run_full.sh

  # 改变场景
  ./run_full.sh --seed 100 --obstacles 10

  # 显示Gazebo GUI + 录制视频
  ./run_full.sh --show-gazebo --record-video

  # 对照测试：控制端用真值
  ./run_full.sh --use-truth

  # 显示LiDAR 2D俯视图（调试用）
  ./run_full.sh --show-lidar-2d
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
        --show-lidar-2d)
            SHOW_LIDAR_2D="true"
            shift
            ;;
        --perception-mode)
            PERCEPTION_MODE="$2"
            shift 2
            ;;
        --use-truth)
            PERCEPTION_MODE="truth"
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

# 检查 baja_cloud_sim-2.1 路径
BAJA_SIM_PATH="/home/yaoh/baja_cloud_sim-2.1"
if [ ! -d "$BAJA_SIM_PATH" ]; then
    echo "错误: 找不到 baja_cloud_sim-2.1 目录"
    echo "请检查路径: $BAJA_SIM_PATH"
    exit 1
fi

# 打印配置
echo "========================================="
echo "完整系统启动"
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
echo "  LiDAR 2D窗口: $([ "$SHOW_LIDAR_2D" = "true" ] && echo "显示" || echo "隐藏")"
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

if [ "$PERCEPTION_MODE" = "lidar" ] || [ "$PERCEPTION_MODE" = "hybrid" ]; then
    SIM_CMD="$SIM_CMD --use-lidar-perception"
fi

echo "启动命令: $SIM_CMD"
echo ""

# 如果需要显示LiDAR 2D窗口，给出提示
if [ "$SHOW_LIDAR_2D" = "true" ]; then
    echo "========================================="
    echo "提示: LiDAR 2D窗口需要在新终端启动"
    echo "等待仿真启动完成后，在新终端运行:"
    echo ""
    echo "  cd ~/lidar3d_ws"
    echo "  source install/setup.bash"
    echo "  ros2 launch lidar3d_bringup play_and_viz.launch.py \\"
    echo "    input_source:=simulation \\"
    echo "    use_surface_detector:=true \\"
    echo "    use_lidar_perception:=true \\"
    echo "    perception_mode:=$PERCEPTION_MODE \\"
    echo "    use_rviz_proc:=true \\"
    echo "    use_rviz_raw:=false"
    echo ""
    echo "========================================="
    echo "按回车继续启动仿真..."
    read
fi

# 启动
exec $SIM_CMD
