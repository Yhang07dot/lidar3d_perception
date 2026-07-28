#!/usr/bin/env python3
"""
Dynamic TF bridge — publishes a fixed transform at a configurable rate on /tf.

Unlike static_transform_publisher (which publishes to /tf_static), this node
publishes to /tf so that rviz2 can correctly render point clouds that depend
on dynamic+static TF chains.

Usage:
  ros2 run lidar3d_bringup tf_bridge \
    --ros-args -p parent_frame:=base_link \
    -p child_frame:=baja_vehicle/base_link/lidar \
    -p x:=0.5 -p z:=1.5 -p rate:=10.0

Added 2026-07-29: Simulation mode TF bridge (base_link → sensor frame).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TfBridge(Node):
    """Publish a fixed transform at a regular interval on /tf."""

    def __init__(self):
        super().__init__('tf_bridge')

        # --- frame names ---
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'baja_vehicle/base_link/lidar')
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # --- transform values ---
        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.5)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        # --- publish rate ---
        self.declare_parameter('rate', 10.0)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Build the transform once (it never changes)
        self._transform = TransformStamped()
        self._transform.header.frame_id = self.parent_frame
        self._transform.child_frame_id = self.child_frame
        self._transform.transform.translation.x = self.get_parameter('x').value
        self._transform.transform.translation.y = self.get_parameter('y').value
        self._transform.transform.translation.z = self.get_parameter('z').value
        # roll/pitch/yaw = 0 → identity quaternion
        self._transform.transform.rotation.x = 0.0
        self._transform.transform.rotation.y = 0.0
        self._transform.transform.rotation.z = 0.0
        self._transform.transform.rotation.w = 1.0

        self.get_logger().info(
            f'TF Bridge: {self.parent_frame} → {self.child_frame} '
            f'@ ({self._transform.transform.translation.x:.2f}, '
            f'{self._transform.transform.translation.y:.2f}, '
            f'{self._transform.transform.translation.z:.2f}) '
            f'{self.get_parameter("rate").value} Hz'
        )

        # Publish at fixed rate on /tf
        period = 1.0 / self.get_parameter('rate').value
        self._timer = self.create_timer(period, self._publish)

    def _publish(self):
        self._transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self._transform)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
