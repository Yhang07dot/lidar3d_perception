from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = 'lidar_cluster'
    pkg_dir = get_package_share_directory(pkg_name)

    return LaunchDescription([
        Node(
            package='rviz2',
            # namespace='',
            executable='rviz2',
            arguments=['-d', [os.path.join(pkg_dir, 'config', 'rviz02.rviz')]]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='center1_os_front_tf_publisher',
            output='screen',
            arguments=['0.75', '0.0', '1.91','0', '0', '0', 'lexus3/base_link', 'lexus3/os_center_a_laser_data_frame'],
        ),
    ])
