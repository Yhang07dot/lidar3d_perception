#!/usr/bin/env python3
"""
最小化感知链启动（配合 baja_cloud_sim-2.2 使用）。

用法:
  cd lidar3d_ws
  source install/setup.bash
  ros2 launch lidar3d_bringup lidar_sim.launch.py

工作流:
  Gazebo gpu_lidar → /lidar/points → filter → patchworkpp →
  surface_detector (C++) → obstacle_adapter → /obstacle_markers

frenet_planner_node 自动订阅 /obstacle_markers 获取障碍物。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('lidar3d_bringup')

    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file == '__default__':
        params_file = os.path.join(pkg_share, 'config', 'lidar_params.yaml')
    else:
        params_file = os.path.expanduser(params_file)

    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)
    sdf_sensor_frame = LaunchConfiguration('sdf_sensor_frame').perform(context)
    target_frame = LaunchConfiguration('target_frame').perform(context)

    # TF bridge: base_link → LiDAR sensor frame (offsets from SDF)
    sensor_tf_node = Node(
        package='lidar3d_bringup', executable='tf_bridge',
        name='sensor_tf_bridge', output='screen',
        parameters=[{
            'use_sim_time': True,
            'parent_frame': target_frame,
            'child_frame': sdf_sensor_frame,
            'x': 0.5, 'y': 0.0, 'z': 1.5,
            'rate': 10.0,
        }],
    )

    # Point cloud filter
    filter_node = Node(
        package='lidar3d_bringup', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('input_cloud', cloud_topic)],
    )

    # Ground segmentation (Patchwork++)
    patch_node = Node(
        package='patchworkpp', executable='patchworkpp_node',
        name='patchworkpp_node', output='screen',
        remappings=[('pointcloud_topic', '/cx/lslidar_point_cloud_filtered')],
        parameters=[{
            'use_sim_time': True,
            'base_frame': sdf_sensor_frame,
            'sensor_height': 1.5,
            'max_range': 50.0,
            'min_range': 1.0,
            'verbose': False,
        }],
    )

    # Surface-fitting obstacle detector (C++)
    surface_detector_node = Node(
        package='lidar3d_perception_cpp', executable='surface_detector_node',
        name='surface_detector', output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # Obstacle adapter: 5-class → 2-class + TF → /obstacle_markers
    adapter_node = Node(
        package='lidar3d_bringup', executable='obstacle_adapter',
        name='obstacle_adapter', output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'source_frame': sdf_sensor_frame,
                'target_frame': target_frame,
                'input_topic': '/obstacles/boxes_3d_surface',
                'passthrough': True,
            }
        ],
    )

    return [
        sensor_tf_node,
        filter_node,
        patch_node,
        surface_detector_node,
        adapter_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='__default__',
            description='Path to lidar_params.yaml'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/points',
            description='Input PointCloud2 topic from Gazebo'),
        DeclareLaunchArgument('sdf_sensor_frame',
            default_value='baja_vehicle/base_link/lidar',
            description='LiDAR sensor frame (matches SDF sensor parent frame)'),
        DeclareLaunchArgument('target_frame', default_value='base_link',
            description='Vehicle base_link frame for output markers'),
        DeclareLaunchArgument('perception_mode', default_value='lidar',
            description='Topic routing mode (kept for compatibility)'),
        OpaqueFunction(function=launch_setup),
    ])
