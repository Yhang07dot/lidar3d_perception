#!/usr/bin/env python3
"""
TF Publisher for 3D LiDAR rosbag playback.

Publishes:
  - odom -> base_link:  dynamically from /chcnav/odom (Chcnav GNSS+INS odometry)
  - base_link -> laser_link:  static (ESTIMATED — replace with calibration)
  - base_link -> imu_link:    static (ESTIMATED — replace with calibration)
  - base_link -> gps_link:    static (ESTIMATED — replace with calibration)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def make_static_tf(parent, child, x, y, z, roll, pitch, yaw):
    """Build a static TransformStamped."""
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    # roll-pitch-yaw -> quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy

    return t


class TfPublisher(Node):
    """Publish odom->base_link from Chcnav odometry + static sensor extrinsics."""

    def __init__(self):
        super().__init__('tf_publisher')

        # Broadcaster for dynamic odom->base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Broadcaster for static sensor extrinsics
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Subscribe to Chcnav odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/chcnav/odom',
            self.odom_callback,
            10,
        )

        # ——— Static extrinsics (ESTIMATED — REPLACE WITH REAL CALIBRATION) ———
        # base_link origin: rear axle center, ground level, x-forward, y-left, z-up
        self.static_transforms = [
            # LiDAR: mounted on top of roll cage, roughly centered longitudinally
            make_static_tf('base_link', 'laser_link',
                           x=0.5, y=0.0, z=1.5, roll=0.0, pitch=0.0, yaw=0.0),
            # IMU: near vehicle centre of gravity, inside chassis
            make_static_tf('base_link', 'imu_link',
                           x=0.3, y=0.0, z=0.4, roll=0.0, pitch=0.0, yaw=0.0),
            # GPS antenna: on top of roll cage, rear
            make_static_tf('base_link', 'gps_link',
                           x=-0.2, y=0.0, z=1.8, roll=0.0, pitch=0.0, yaw=0.0),
        ]

        # Publish static transforms once (StaticTransformBroadcaster handles latching)
        self.static_broadcaster.sendTransform(self.static_transforms)
        self.get_logger().info(
            'Published static TFs (ESTIMATED values — update with real calibration): '
            'base_link -> laser_link, imu_link, gps_link'
        )

        self.get_logger().info('TF Publisher ready, waiting for /chcnav/odom ...')

    def odom_callback(self, msg: Odometry):
        """Convert odometry pose to odom -> base_link TF and publish."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id       # 'odom'
        t.child_frame_id = msg.child_frame_id          # 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
