#!/usr/bin/env python3
"""
Launch file for rosbag playback, TF, filter, ground segmentation, and rviz2.

Usage:
    ros2 launch lidar3d_bringup play_and_viz.launch.py
    ros2 launch lidar3d_bringup play_and_viz.launch.py sensor_height:=1.2
    ros2 launch lidar3d_bringup play_and_viz.launch.py enable_ground_seg:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)
    loop = LaunchConfiguration('loop').perform(context)
    sensor_height = LaunchConfiguration('sensor_height').perform(context)

    bag_dir = os.path.expanduser(bag_dir)

    pkg_share = get_package_share_directory('lidar3d_bringup')
    rviz_raw = os.path.join(pkg_share, 'rviz', 'lidar3d_raw.rviz')
    rviz_proc = os.path.join(pkg_share, 'rviz', 'lidar3d_processed.rviz')

    # Build rosbag play command
    rosbag_cmd = [
        'ros2', 'bag', 'play', bag_dir,
        '--rate', rate,
        '--start-offset', start_offset,
        '--clock',
        '--read-ahead-queue-size', '200',
    ]
    if loop.lower() == 'true':
        rosbag_cmd.append('--loop')

    # --- Build filter node params ---
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

    return [
        # --- rosbag play ---
        ExecuteProcess(
            cmd=rosbag_cmd,
            output='screen',
        ),

        # --- TF publisher ---
        Node(
            package='lidar3d_bringup',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # --- PointCloud filter (distance + height) ---
        Node(
            package='lidar3d_bringup',
            executable='pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
            parameters=[filter_params],
        ),

        # --- Patchwork++ ground segmentation ---
        Node(
            package='patchworkpp',
            executable='patchworkpp_node',
            name='patchworkpp_node',
            output='screen',
            remappings=[('pointcloud_topic', '/cx/lslidar_point_cloud_filtered')],
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'laser_link',
                'sensor_height': float(sensor_height),
                'max_range': 50.0,
                'min_range': 1.0,
                'verbose': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_ground_seg')),
        ),

        # --- Obstacle clustering (euclidean_grid) ---
        Node(
            package='lidar_cluster',
            executable='euclidean_grid',
            name='euclidean_grid',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'points_in_topic': '/patchworkpp/nonground',
                'points_out_topic': '/clusters/points',
                'marker_out_topic': '/clusters/markers',
                'tolerance': 0.5,
                'voxel_leaf_size': 0.1,
                'min_points_number_per_voxel': 3,
                'max_cluster_size': 50000,
                'verbose1': False,
                'verbose2': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_ground_seg')),
        ),

        # --- Real bounding boxes from /clusters/points ---
        Node(
            package='lidar3d_bringup',
            executable='cluster_bbox',
            name='cluster_bbox',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('enable_ground_seg')),
        ),

        # --- rviz2 窗口1: 原始过滤点云（参考窗）---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_raw',
            arguments=['-d', rviz_raw],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # --- rviz2 窗口2: 处理后点云（检测窗，地面绿+障碍红）---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_proc',
            arguments=['-d', rviz_proc],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_dir',
            default_value=os.path.expanduser('~/lidar3d_ws/bags'),
            description='Path to rosbag2 directory',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='Playback rate multiplier',
        ),
        DeclareLaunchArgument(
            'start_offset',
            default_value='0.0',
            description='Seconds to skip from beginning',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Loop playback (true/false)',
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='__default__',
            description='Filter max distance (m)',
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='__default__',
            description='Filter min distance (m)',
        ),
        DeclareLaunchArgument(
            'min_height',
            default_value='__default__',
            description='Filter min Z height (m)',
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='__default__',
            description='Filter max Z height (m)',
        ),
        DeclareLaunchArgument(
            'sensor_height',
            default_value='1.5',
            description='LiDAR height above ground (m) for Patchwork++',
        ),
        DeclareLaunchArgument(
            'enable_ground_seg',
            default_value='true',
            description='Enable Patchwork++ ground segmentation (true/false)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
