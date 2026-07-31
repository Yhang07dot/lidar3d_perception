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
    # --- Load YAML config ---
    pkg_share = get_package_share_directory('lidar3d_bringup')
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file == '__default__':
        params_file = os.path.join(pkg_share, 'config', 'lidar_params.yaml')
    else:
        params_file = os.path.expanduser(params_file)

    # --- source params ---
    src = LaunchConfiguration('input_source').perform(context)
    raw_topic = LaunchConfiguration('cloud_topic').perform(context)
    # resolve __auto__ sentinel
    if raw_topic == '__auto__':
        cloud_topic = '/cx/lslidar_point_cloud' if src == 'rosbag' else '/lidar/points'
    else:
        cloud_topic = raw_topic

    # 2026-07-29: sim_time only for rosbag and simulation; real lidar uses wall clock
    use_sim_time_val = src in ('rosbag', 'simulation')
    # 2026-07-29: 3D clustering pipeline (parallel to 2D, for slope-obstacle discrimination)
    use_lidar_percep = LaunchConfiguration('use_lidar_perception').perform(context).lower() == 'true'
    use_voxel = LaunchConfiguration('use_voxel_analyzer').perform(context).lower() == 'true'
    use_surface = LaunchConfiguration('use_surface_detector').perform(context).lower() == 'true'

    # Use_ ROS bag params (only in rosbag mode)
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)
    loop = LaunchConfiguration('loop').perform(context)

    # Patchwork++ param
    sensor_height = LaunchConfiguration('sensor_height').perform(context)

    bag_dir = os.path.expanduser(bag_dir)

    rviz_raw = os.path.join(pkg_share, 'rviz', 'lidar3d_raw.rviz')
    rviz_proc = os.path.join(pkg_share, 'rviz', 'lidar3d_processed.rviz')
    rviz_voxel = os.path.join(pkg_share, 'rviz', 'lidar3d_voxel.rviz')
    rviz_surface = os.path.join(pkg_share, 'rviz', 'lidar3d_surface_2d.rviz')

    # --- filter params ---
    filter_params = {'use_sim_time': use_sim_time_val}
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
        parameters=[{'use_sim_time': use_sim_time_val}],
    )

    # --- TF bridges (sim/lidar modes) ---
    # 2026-07-29: dynamic TF node for sensor frame (publishes to /tf, not /tf_static).
    # rviz2 requires /tf (not /tf_static) for point cloud rendering with mixed
    # dynamic+static TF chains.
    sensor_tf_node = Node(
        package='lidar3d_bringup', executable='tf_bridge',
        name='sensor_tf_bridge', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_val,
            'parent_frame': 'base_link',
            'child_frame': 'baja_vehicle/base_link/lidar',
            'x': 0.5, 'y': 0.0, 'z': 1.5,
            'rate': 10.0,
        }],
    )
    # odom → map (rviz2 Fixed Frame = odom, simulation uses map).
    # Static TF is fine here — not directly involved in point cloud rendering.
    odom_tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_map_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
        parameters=[{'use_sim_time': use_sim_time_val}],
    )

    # --- filter (remap input_cloud → cloud_topic) ---
    filter_node = Node(
        package='lidar3d_bringup', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[params_file, filter_params],
        remappings=[('input_cloud', cloud_topic)],
    )

    seg_enabled = IfCondition(LaunchConfiguration('enable_ground_seg'))

    # --- Patchwork++ (subscribes to filtered output) ---
    patch_node = Node(
        package='patchworkpp', executable='patchworkpp_node',
        name='patchworkpp_node', output='screen',
        remappings=[('pointcloud_topic', '/cx/lslidar_point_cloud_filtered')],
        parameters=[{
            'use_sim_time': use_sim_time_val,
            # 2026-07-29: sim mode uses Gazebo frame directly; rosbag keeps laser_link
            'base_frame': 'laser_link' if is_rosbag else 'baja_vehicle/base_link/lidar',
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
            'use_sim_time': use_sim_time_val,
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
        parameters=[{'use_sim_time': use_sim_time_val}],
        condition=seg_enabled,
    )

    # --- classify + transform → /obstacle_markers ---
    # resolve input topic: voxel > 2D
    if use_surface:
        adapter_input = '/obstacles/boxes_3d_surface'
    elif use_voxel:
        adapter_input = '/obstacles/boxes_3d_voxel'
    else:
        adapter_input = '/obstacles/boxes'

    adapter_node = Node(
        package='lidar3d_bringup', executable='obstacle_adapter',
        name='obstacle_adapter', output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time_val,
                'source_frame': 'laser_link' if is_rosbag else 'baja_vehicle/base_link/lidar',
                'input_topic': adapter_input,
                'passthrough': use_voxel or use_surface,
            }
        ],
        condition=seg_enabled,
    )

    # --- LiDAR perception replacing truth data (controlled by use_lidar_perception) ---
    lidar_percep_cond = IfCondition(LaunchConfiguration('use_lidar_perception'))

    # --- 2026-07-30: voxel-based analyser (parallel to cluster_analyzer) ---
    voxel_cond = IfCondition(LaunchConfiguration('use_voxel_analyzer'))

    voxel_node = Node(
        package='lidar3d_bringup', executable='voxel_analyzer',
        name='voxel_analyzer', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time_val}],
        condition=voxel_cond,
    )

    road_node = Node(
        package='lidar3d_bringup', executable='road_analyzer',
        name='road_analyzer', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time_val}],
        condition=lidar_percep_cond,
    )

    # --- 2026-07-31: surface-fitting detector ---
    surface_node = Node(
        package='lidar3d_bringup', executable='surface_detector',
        name='surface_detector', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time_val}],
        condition=IfCondition(LaunchConfiguration('use_surface_detector')),
    )

    # --- rviz2 ---
    rviz_raw_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_raw',
        arguments=['-d', rviz_raw],
        parameters=[{'use_sim_time': use_sim_time_val}], output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz_raw')),
    )
    rviz_proc_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_proc',
        arguments=['-d', rviz_proc],
        parameters=[{'use_sim_time': use_sim_time_val}], output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz_proc')),
    )
    rviz_voxel_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_voxel',
        arguments=['-d', rviz_voxel],
        parameters=[{'use_sim_time': use_sim_time_val}], output='screen',
        condition=voxel_cond,
    )
    rviz_surface_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_surface',
        arguments=['-d', rviz_surface],
        parameters=[{'use_sim_time': use_sim_time_val}], output='screen',
        condition=IfCondition(LaunchConfiguration('use_surface_detector')),
    )

    # --- assembly ---
    nodes = []

    # rosbag mode only
    if is_rosbag:
        nodes.append(rosbag_node)
        nodes.append(tf_node)
    else:
        # sim/lidar modes: bridge frames (2026-07-29: dynamic sensor TF + static odom→map)
        nodes.extend([sensor_tf_node, odom_tf_node])

    # always
    nodes.append(filter_node)
    nodes.extend([patch_node, cluster_node, bbox_node, adapter_node])
    # 2026-07-29: LiDAR perception replacing truth data
    nodes.append(road_node)
    # 2026-07-30: voxel analyser (parallel to cluster_analyzer)
    nodes.append(voxel_node)
    nodes.append(surface_node)
    nodes.extend([rviz_raw_node, rviz_proc_node, rviz_voxel_node, rviz_surface_node])

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_source', default_value='rosbag',
            description="'rosbag' | 'simulation' | 'lidar'"),

        # --- Parameter file ---
        DeclareLaunchArgument('params_file', default_value='__default__',
            description='Path to YAML parameter file (default: config/lidar_params.yaml)'),

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
        # 2026-07-29: 3D clustering pipeline for slope-obstacle discrimination
        DeclareLaunchArgument('use_lidar_perception', default_value='false',
            description='Replace truth_perception data with LiDAR-based road boundaries+obstacles'),
        DeclareLaunchArgument('use_voxel_analyzer', default_value='false',
            description='Use voxel-grid analyser instead of PCA-on-clusters'),
        DeclareLaunchArgument('use_surface_detector', default_value='false',
            description='Use terrain-surface fitting detector (recommended)'),
        DeclareLaunchArgument('use_rviz_raw', default_value='true',
            description='Show raw filtered point cloud rviz2 window'),
        DeclareLaunchArgument('use_rviz_proc', default_value='true',
            description='Show 2D pipeline processed rviz2 window'),
        OpaqueFunction(function=launch_setup),
    ])
