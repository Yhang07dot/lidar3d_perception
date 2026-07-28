#!/usr/bin/env python3
"""
Obstacle adapter: classify + transform + republish for planning/control.

Subscribes to /obstacles/boxes (CUBE MarkerArray in laser_link from cluster_bbox).
Classifies each obstacle by 3D shape, transforms to base_link via TF,
and publishes to /obstacle_markers (format expected by baja_cloud_sim planner).

Classifications:
  0 = generic obstacle
  1 = pole      (tall, thin; height / width > 2.0)
  2 = bump      (low, flat; height < 0.25m)
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


def classify_obstacle(scale):
    """
    Classify obstacle by bounding box shape.
    Returns (label_str, type_id).
    """
    dims = sorted([scale.x, scale.y, scale.z])  # ascending
    d_min, d_mid, d_max = dims  # smallest, middle, largest dimension

    height = d_max
    width = d_mid if d_mid > 0.01 else 0.01

    # Low and flat → bump / speed bump
    if height < 0.25:
        return "bump", 2

    # Tall and thin → pole
    aspect_ratio = height / width
    if height > 0.3 and aspect_ratio > 2.0:
        return "pole", 1

    return "generic", 0


class ObstacleAdapter(Node):
    """Classify obstacles and republish for planning/control."""

    def __init__(self):
        super().__init__('obstacle_adapter')

        # TF for laser_link → base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to our obstacle boxes
        self.sub = self.create_subscription(
            MarkerArray,
            '/obstacles/boxes',
            self.callback,
            10,
        )

        # Publish in planning/control format
        self.pub = self.create_publisher(
            MarkerArray,
            '/obstacle_markers',
            10,
        )

        self._frame_count = 0
        self._log_interval = 10

        self.get_logger().info(
            'Obstacle Adapter ready — /obstacles/boxes → /obstacle_markers'
        )

    def callback(self, msg: MarkerArray):
        self._frame_count += 1

        # Look up laser_link → base_link transform
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'laser_link', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            label, type_id = classify_obstacle(marker.scale)

            # Transform position from laser_link → base_link
            # T_base_laser = (tx, ty, tz, qx, qy, qz, qw)
            tr = t.transform.translation
            q = t.transform.rotation
            px, py, pz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z

            # Apply rotation then translation
            # p_base = R_laser→base_link * p_laser + t_laser→base_link
            # Using quaternion rotation
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            # Rotate point: p' = q * p * q_conj
            # Simplified: p' = p + 2 * cross(q.xyz, cross(q.xyz, p) + q.w * p)
            cx = 2.0 * (qy * pz - qz * py)
            cy = 2.0 * (qz * px - qx * pz)
            cz = 2.0 * (qx * py - qy * px)
            rx = px + qw * cx + (qy * cz - qz * cy)
            ry = py + qw * cy + (qz * cx - qx * cz)
            rz = pz + qw * cz + (qx * cy - qy * cx)

            base_x = rx + tr.x
            base_y = ry + tr.y
            base_z = rz + tr.z

            # Create CUBE marker in base_link
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = now
            m.ns = label
            m.id = marker.id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = base_x
            m.pose.position.y = base_y
            m.pose.position.z = base_z
            m.pose.orientation.w = 1.0
            m.scale = marker.scale
            m.color.r = 1.0 if type_id == 1 else 0.0    # red for poles
            m.color.g = 0.0 if type_id == 1 else 1.0    # green for others
            m.color.b = 0.0
            m.color.a = 0.6
            m.lifetime.nanosec = 200_000_000  # 200ms
            out.markers.append(m)

        self.pub.publish(out)

        if self._frame_count % self._log_interval == 0:
            types = {}
            for m in out.markers:
                types[m.ns] = types.get(m.ns, 0) + 1
            summary = ", ".join(f"{k}={v}" for k, v in types.items())
            self.get_logger().info(
                f'Frame {self._frame_count}: {len(out.markers)} obstacles → {summary}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
