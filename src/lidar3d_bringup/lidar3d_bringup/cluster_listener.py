#!/usr/bin/env python3
"""
Listener for /clusters/markers — prints valid obstacle detection results.

Filters out DELETE markers, TEXT labels, debug cylinders, and origin ghost data.
Only prints CYLINDER markers with ns="cluster_center" (actual obstacle centroids).

Run standalone:
    ros2 run lidar3d_bringup cluster_listener --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ClusterListener(Node):
    """Prints valid cluster centroid info, filtering out debug/delete markers."""

    def __init__(self):
        super().__init__('cluster_listener')

        self.sub = self.create_subscription(
            MarkerArray,
            '/clusters/markers',
            self.callback,
            10,
        )

        self._frame_count = 0
        self._log_interval = 10

        self.get_logger().info('Cluster Listener ready — waiting for /clusters/markers ...')

    @staticmethod
    def _is_valid_obstacle(marker: Marker) -> bool:
        """Keep only ADD-action CYLINDER markers with ns='cluster_center' at non-origin pos."""
        if marker.action != Marker.ADD:
            return False
        if marker.type != Marker.CYLINDER:
            return False
        if marker.ns != 'cluster_center':
            return False
        p = marker.pose.position
        if abs(p.x) < 0.01 and abs(p.y) < 0.01:
            return False
        return True

    def callback(self, msg: MarkerArray):
        self._frame_count += 1

        # Filter valid obstacles
        valid = [m for m in msg.markers if self._is_valid_obstacle(m)]

        if self._frame_count % self._log_interval == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'{len(msg.markers)} raw markers → {len(valid)} valid obstacles'
            )
            for marker in valid:
                p = marker.pose.position
                self.get_logger().info(
                    f'  id={marker.id} '
                    f'pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})  '
                    f'| 注: 尺寸 0.8 为固定可视化标记, 非真实包围框'
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
