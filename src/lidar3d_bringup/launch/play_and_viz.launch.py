#!/usr/bin/env python3
"""
Launch file for rosbag playback with TF publishing and rviz2 visualization.

Usage:
    ros2 launch lidar3d_bringup play_and_viz.launch.py

Arguments:
    bag_dir:  path to rosbag directory (default: ~/lidar3d_ws/bags)
    rate:     playback rate (default: 1.0)
    start_offset: seconds to skip from beginning (default: 0.0)
    use_sim_time: use /clock from rosbag (default: True)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)
    loop = LaunchConfiguration('loop').perform(context)
    max_range = LaunchConfiguration('max_range').perform(context)
    min_range = LaunchConfiguration('min_range').perform(context)
    min_height = LaunchConfiguration('min_height').perform(context)
    max_height = LaunchConfiguration('max_height').perform(context)

    bag_dir = os.path.expanduser(bag_dir)

    pkg_share = get_package_share_directory('lidar3d_bringup')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lidar3d.rviz')

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

        # --- PointCloud filter (range + height) ---
        Node(
            package='lidar3d_bringup',
            executable='pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_range': float(max_range),
                'min_range': float(min_range),
                'min_height': float(min_height),
                'max_height': float(max_height),
            }],
        ),

        # --- rviz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
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
            default_value='50.0',
            description='Max detection distance (m)',
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='0.5',
            description='Min detection distance (m), filters self-vehicle',
        ),
        DeclareLaunchArgument(
            'min_height',
            default_value='-2.0',
            description='Min Z height in laser_link frame (m)',
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='3.0',
            description='Max Z height in laser_link frame (m)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
