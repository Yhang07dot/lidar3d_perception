#!/usr/bin/env python3
"""
Listener for /clusters/markers — prints obstacle detection results.

Run standalone:
    ros2 run lidar3d_bringup cluster_listener --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class ClusterListener(Node):
    """Prints cluster bounding box info from euclidean_grid output."""

    def __init__(self):
        super().__init__('cluster_listener')

        self.sub = self.create_subscription(
            MarkerArray,
            '/clusters/markers',
            self.callback,
            10,
        )

        self._frame_count = 0
        self._log_interval = 10  # log every N frames

        self.get_logger().info('Cluster Listener ready — waiting for /clusters/markers ...')

    def callback(self, msg: MarkerArray):
        self._frame_count += 1
        n = len(msg.markers)

        if self._frame_count % self._log_interval == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: {n} obstacles detected'
            )
            for i, marker in enumerate(msg.markers):
                p = marker.pose.position
                s = marker.scale
                self.get_logger().info(
                    f'  [{i}] id={marker.id} '
                    f'pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) '
                    f'size=({s.x:.2f}, {s.y:.2f}, {s.z:.2f})'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ClusterListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
