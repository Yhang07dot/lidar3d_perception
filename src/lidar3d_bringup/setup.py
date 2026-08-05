from setuptools import find_packages, setup
import glob

package_name = 'lidar3d_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz',
            glob.glob('rviz/*.rviz')),
        ('share/' + package_name + '/config',
            glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaoh',
    maintainer_email='1253063983@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_publisher = lidar3d_bringup.tf_publisher:main',
            'tf_bridge = lidar3d_bringup.tf_bridge:main',
            'pointcloud_filter = lidar3d_bringup.pointcloud_filter:main',
            'cluster_bbox = lidar3d_bringup.cluster_bbox:main',
            'obstacle_adapter = lidar3d_bringup.obstacle_adapter:main',
            'road_analyzer = lidar3d_bringup.road_analyzer:main',
        ],
    },
)
