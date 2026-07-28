#!/usr/bin/env python3
"""
Perception pipeline launch — 支持三种数据源切换.

# rosbag 回放（默认）
ros2 launch lidar3d_bringup play_and_viz.launch.py

# 仿真 LiDAR (BajaSimPart)
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=simulation

# 实车 LiDAR 驱动
ros2 launch lidar3d_bringup play_and_viz.launch.py input_source:=lidar

# 自定义话题 + 关地面分割
ros2 launch lidar3d_bringup play_and_viz.launch.py \
    input_source:=simulation cloud_topic:=/my_lidar enable_ground_seg:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # --- source params ---
    src = LaunchConfiguration('input_source').perform(context)
    raw_topic = LaunchConfiguration('cloud_topic').perform(context)
    # resolve __auto__ sentinel
    if raw_topic == '__auto__':
        cloud_topic = '/cx/lslidar_point_cloud' if src == 'rosbag' else '/lidar/points'
    else:
        cloud_topic = raw_topic

    # Use_ ROS bag params (only in rosbag mode)
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)
    loop = LaunchConfiguration('loop').perform(context)

    # Patchwork++ param
    sensor_height = LaunchConfiguration('sensor_height').perform(context)

    bag_dir = os.path.expanduser(bag_dir)

    pkg_share = get_package_share_directory('lidar3d_bringup')
    rviz_raw = os.path.join(pkg_share, 'rviz', 'lidar3d_raw.rviz')
    rviz_proc = os.path.join(pkg_share, 'rviz', 'lidar3d_processed.rviz')

    # --- filter params ---
    filter_params = {'use_sim_time': True}
    overrides = {
        'max_range': LaunchConfiguration('max_range').perform(context),
        'min_range': LaunchConfiguration('min_range').perform(context),
        'min_height': LaunchConfiguration('min_height').perform(context),
        'max_height': LaunchConfiguration('max_height').perform(context),
    }
    for key, val in overrides.items():
        if val != '__default__':
            filter_params[key] = float(val)

    # -- rosbag play (only in rosbag mode) ---
    is_rosbag = (src == 'rosbag')
    rosbag_cmd = [
        'ros2', 'bag', 'play', bag_dir,
        '--rate', rate, '--start-offset', start_offset,
        '--clock', '--read-ahead-queue-size', '200',
    ]
    if loop.lower() == 'true':
        rosbag_cmd.append('--loop')

    rosbag_node = ExecuteProcess(cmd=rosbag_cmd, output='screen')

    # --- TF publisher (only in rosbag mode) ---
    tf_node = Node(
        package='lidar3d_bringup', executable='tf_publisher',
        name='tf_publisher', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- static TF: Gazebo frame → laser_link (sim/lidar modes) ---
    static_tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0',
                   'baja_vehicle/base_link/lidar', 'laser_link'],
        parameters=[{'use_sim_time': True}],
    )

    # --- filter (remap input_cloud → cloud_topic) ---
    filter_node = Node(
        package='lidar3d_bringup', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[filter_params],
        remappings=[('input_cloud', cloud_topic)],
    )

    seg_enabled = IfCondition(LaunchConfiguration('enable_ground_seg'))

    # --- Patchwork++ (subscribes to filtered output) ---
    patch_node = Node(
        package='patchworkpp', executable='patchworkpp_node',
        name='patchworkpp_node', output='screen',
        remappings=[('pointcloud_topic', '/cx/lslidar_point_cloud_filtered')],
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'laser_link',
            'sensor_height': float(sensor_height),
            'max_range': 50.0, 'min_range': 1.0, 'verbose': False,
        }],
        condition=seg_enabled,
    )

    # --- euclidean clustering ---
    cluster_node = Node(
        package='lidar_cluster', executable='euclidean_grid',
        name='euclidean_grid', output='screen',
        parameters=[{
            'use_sim_time': True,
            'points_in_topic': '/patchworkpp/nonground',
            'points_out_topic': '/clusters/points',
            'marker_out_topic': '/clusters/markers',
            'tolerance': 0.5, 'voxel_leaf_size': 0.1,
            'min_points_number_per_voxel': 3, 'max_cluster_size': 50000,
            'verbose1': False, 'verbose2': False,
        }],
        condition=seg_enabled,
    )

    # --- bounding boxes ---
    bbox_node = Node(
        package='lidar3d_bringup', executable='cluster_bbox',
        name='cluster_bbox', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=seg_enabled,
    )

    # --- classify + transform → /obstacle_markers ---
    adapter_node = Node(
        package='lidar3d_bringup', executable='obstacle_adapter',
        name='obstacle_adapter', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=seg_enabled,
    )

    # --- rviz2 ---
    rviz_raw_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_raw',
        arguments=['-d', rviz_raw],
        parameters=[{'use_sim_time': True}], output='screen',
    )
    rviz_proc_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_proc',
        arguments=['-d', rviz_proc],
        parameters=[{'use_sim_time': True}], output='screen',
    )

    # --- assembly ---
    nodes = []

    # rosbag mode only
    if is_rosbag:
        nodes.append(rosbag_node)
        nodes.append(tf_node)
    else:
        # sim/lidar modes: bridge Gazebo frame → laser_link
        nodes.append(static_tf_node)

    # always
    nodes.append(filter_node)
    nodes.extend([patch_node, cluster_node, bbox_node, adapter_node])
    nodes.extend([rviz_raw_node, rviz_proc_node])

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_source', default_value='rosbag',
            description="'rosbag' | 'simulation' | 'lidar'"),

        # --- cloud_topic — auto-set per input_source ---
        DeclareLaunchArgument('cloud_topic', default_value='__auto__',
            description='Input PointCloud2 topic (auto: rosbag→/cx/..., sim→/lidar/...)'),

        # --- rosbag args ---
        DeclareLaunchArgument('bag_dir',
            default_value=os.path.expanduser('~/lidar3d_ws/bags'),
            description='[rosbag] Path to rosbag2 directory'),
        DeclareLaunchArgument('rate', default_value='1.0',
            description='[rosbag] Playback rate'),
        DeclareLaunchArgument('start_offset', default_value='0.0',
            description='[rosbag] Seconds to skip'),
        DeclareLaunchArgument('loop', default_value='true',
            description='[rosbag] Loop playback'),

        # --- filter args ---
        DeclareLaunchArgument('max_range', default_value='__default__',
            description='Filter max distance (m)'),
        DeclareLaunchArgument('min_range', default_value='__default__',
            description='Filter min distance (m)'),
        DeclareLaunchArgument('min_height', default_value='__default__',
            description='Filter min Z (m)'),
        DeclareLaunchArgument('max_height', default_value='__default__',
            description='Filter max Z (m)'),

        # --- sensor ---
        DeclareLaunchArgument('sensor_height', default_value='1.5',
            description='LiDAR height above ground (m)'),
        DeclareLaunchArgument('enable_ground_seg', default_value='true',
            description='Enable ground segmentation + downstream'),

        OpaqueFunction(function=launch_setup),
    ])
