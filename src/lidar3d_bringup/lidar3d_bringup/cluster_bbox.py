#!/usr/bin/env python3
"""
Real 3D bounding box node.

Subscribes to /clusters/points (PointCloud2 with intensity = cluster ID),
computes AABB (axis-aligned bounding box) per cluster, publishes:
  - /obstacles/markers  (CUBE MarkerArray for rviz2)
  - /obstacles/centers  (SPHERE markers at centroids for rviz2)

The AABB is computed from actual point min/max in (x, y, z) — no hardcoded sizes.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class ClusterBBox(Node):
    """Real bounding boxes from clustered point cloud."""

    def __init__(self):
        super().__init__('cluster_bbox')

        self.sub = self.create_subscription(
            PointCloud2,
            '/clusters/points',
            self.callback,
            10,
        )
        self.bbox_pub = self.create_publisher(MarkerArray, '/obstacles/boxes', 10)
        self.ctr_pub = self.create_publisher(MarkerArray, '/obstacles/centers', 10)

        self._frame_count = 0
        self._log_interval = 10

        self.get_logger().info('Cluster BBox ready — waiting for /clusters/points ...')

    def callback(self, msg: PointCloud2):
        self._frame_count += 1

        # Parse PointCloud2 → numpy: x, y, z, intensity
        # Layout: x(f32,0) y(f32,4) z(f32,8) (pad4) intensity(f32,16), point_step=20
        dt = np.dtype({
            'names': ['x', 'y', 'z', 'intensity'],
            'formats': [np.float32, np.float32, np.float32, np.float32],
            'offsets': [0, 4, 8, 16],
            'itemsize': msg.point_step,
        })
        points = np.frombuffer(msg.data, dtype=dt)
        if len(points) == 0:
            return

        x, y, z = points['x'], points['y'], points['z']
        cid = points['intensity'].astype(np.int32)

        # Group by cluster ID, skip 0 (unclustered noise)
        unique_ids = np.unique(cid)
        valid_ids = unique_ids[(unique_ids > 0)]

        now = msg.header.stamp
        frame_id = msg.header.frame_id  # laser_link

        box_markers = MarkerArray()
        center_markers = MarkerArray()

        for c in valid_ids:
            mask = cid == c
            cx, cy, cz = x[mask], y[mask], z[mask]

            min_pt = np.array([cx.min(), cy.min(), cz.min()])
            max_pt = np.array([cx.max(), cy.max(), cz.max()])
            centroid = (min_pt + max_pt) / 2.0
            dims = max_pt - min_pt

            # Skip degenerate clusters (zero-size or single-point lines)
            if np.all(dims < 0.01):
                continue

            # --- CUBE marker (axis-aligned bounding box) ---
            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = now
            box.ns = 'obstacle_box'
            box.id = int(c)
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(centroid[0])
            box.pose.position.y = float(centroid[1])
            box.pose.position.z = float(centroid[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(max(dims[0], 0.05))
            box.scale.y = float(max(dims[1], 0.05))
            box.scale.z = float(max(dims[2], 0.05))
            box.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)
            box.lifetime.nanosec = 200_000_000  # 200ms, cleared next frame
            box_markers.markers.append(box)

            # --- SPHERE marker (centroid) ---
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = 'obstacle_center'
            sphere.id = int(c)
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(centroid[0])
            sphere.pose.position.y = float(centroid[1])
            sphere.pose.position.z = float(centroid[2])
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            sphere.lifetime.nanosec = 200_000_000
            center_markers.markers.append(sphere)

        # Log
        if self._frame_count % self._log_interval == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'{len(valid_ids)} clusters → {len(box_markers.markers)} valid obstacles'
            )
            for m in box_markers.markers:
                p = m.pose.position
                s = m.scale
                self.get_logger().info(
                    f'  id={m.id} '
                    f'pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) '
                    f'size=({s.x:.2f}, {s.y:.2f}, {s.z:.2f})'
                )

        self.bbox_pub.publish(box_markers)
        self.ctr_pub.publish(center_markers)


def main(args=None):
    rclpy.init(args=args)
    node = ClusterBBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
