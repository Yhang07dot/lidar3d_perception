#!/bin/bash
# LiDAR 3D 感知系统启动脚本
# 用法: ./run_lidar.sh [选项]

set -e

# 默认参数
INPUT_SOURCE="simulation"
USE_SURFACE="true"
USE_LIDAR_PERCEPTION="true"
PERCEPTION_MODE="lidar"
USE_RVIZ_RAW="false"
USE_RVIZ_PROC="true"
ENABLE_GROUND_SEG="true"

# 解析命令行参数
show_help() {
    cat << EOF
LiDAR 3D 感知系统启动脚本

用法: ./run_lidar.sh [选项]

选项:
  --input-source SOURCE    数据源: rosbag | simulation | lidar (默认: simulation)
  --perception-mode MODE   感知模式: lidar | truth | hybrid (默认: lidar)
                          - lidar: LiDAR替换真值，控制端用LiDAR数据
                          - truth: 控制端用真值，LiDAR输出到/lidar/*命名空间
                          - hybrid: 两者并行，独立命名空间
  --no-surface            禁用surface_detector (使用cluster_bbox代替)
  --show-raw              显示原始点云窗口
  --no-2d                 不显示2D俯视图窗口
  --disable-ground-seg    禁用地面分割
  --help, -h              显示此帮助信息

示例:
  # 默认启动 (仿真 + LiDAR感知替换真值 + 2D窗口)
  ./run_lidar.sh

  # 对照测试: 控制端用真值，LiDAR在后台运行
  ./run_lidar.sh --perception-mode truth

  # 调试对比: 两者并行显示
  ./run_lidar.sh --perception-mode hybrid

  # rosbag回放
  ./run_lidar.sh --input-source rosbag

  # 仅显示原始点云，不显示2D窗口
  ./run_lidar.sh --show-raw --no-2d
EOF
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --input-source)
            INPUT_SOURCE="$2"
            shift 2
            ;;
        --perception-mode)
            PERCEPTION_MODE="$2"
            shift 2
            ;;
        --no-surface)
            USE_SURFACE="false"
            shift
            ;;
        --show-raw)
            USE_RVIZ_RAW="true"
            shift
            ;;
        --no-2d)
            USE_RVIZ_PROC="false"
            shift
            ;;
        --disable-ground-seg)
            ENABLE_GROUND_SEG="false"
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

# 检查工作空间
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ ! -f "install/setup.bash" ]; then
    echo "错误: 找不到 install/setup.bash"
    echo "请先编译工作空间: colcon build"
    exit 1
fi

# 加载环境
source install/setup.bash

# 打印配置
echo "========================================="
echo "LiDAR 3D 感知系统启动"
echo "========================================="
echo "输入源:       $INPUT_SOURCE"
echo "感知模式:     $PERCEPTION_MODE"
echo "Surface检测:  $USE_SURFACE"
echo "地面分割:     $ENABLE_GROUND_SEG"
echo "原始点云窗口: $USE_RVIZ_RAW"
echo "2D俯视窗口:   $USE_RVIZ_PROC"
echo "========================================="

# 根据感知模式设置 use_lidar_perception
if [ "$PERCEPTION_MODE" = "lidar" ] || [ "$PERCEPTION_MODE" = "hybrid" ]; then
    USE_LIDAR_PERCEPTION="true"
else
    USE_LIDAR_PERCEPTION="false"
fi

# 启动
echo "启动感知流程..."
exec ros2 launch lidar3d_bringup play_and_viz.launch.py \
    input_source:="$INPUT_SOURCE" \
    use_surface_detector:="$USE_SURFACE" \
    use_lidar_perception:="$USE_LIDAR_PERCEPTION" \
    perception_mode:="$PERCEPTION_MODE" \
    use_rviz_raw:="$USE_RVIZ_RAW" \
    use_rviz_proc:="$USE_RVIZ_PROC" \
    enable_ground_seg:="$ENABLE_GROUND_SEG"
