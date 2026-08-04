#!/usr/bin/env python3
"""
Obstacle adapter: classify + transform + republish for planning/control.

Subscribes to /obstacles/boxes (CUBE MarkerArray from cluster_bbox).
Classifies each obstacle by 3D shape, transforms to base_link via TF,
and publishes to /obstacle_markers (format expected by baja_cloud_sim planner).

Parameters:
  source_frame  — frame of incoming markers (default: 'laser_link')
  target_frame  — frame to transform markers into (default: 'base_link')

Classifications:
  0 = generic obstacle
  1 = pole      (tall, thin; height / width > 2.0)
  2 = bump      (low, flat; height < 0.25m)

Changelog:
  2026-07-29: source_frame/target_frame made configurable for sim mode.
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


# 颜色映射 - 与surface_detector保持一致 (2026-08-03)
TYPE_COLORS = {
    'obstacle': (1.0, 0.0, 0.0, 0.7),      # 红色 - 不可通过
    'passable_low': (0.0, 0.8, 0.0, 0.5),  # 绿色 - 可通过
    'passable_high': (1.0, 0.9, 0.0, 0.6), # 黄色 - 减速通过
    'boundary': (0.0, 0.5, 1.0, 0.6),      # 蓝色 - 路沿
    'unknown': (0.7, 0.7, 0.7, 0.4),       # 灰色 - 未知
}


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

        # 2026-07-29: source/target frame now configurable for sim vs rosbag modes
        self.declare_parameter('source_frame', 'laser_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('input_topic', '/obstacles/boxes')  # 2026-07-29: switch 2D/3D pipeline
        self.declare_parameter('passthrough', False)               # 2026-07-29: preserve 3D pipeline classification
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.input_topic = self.get_parameter('input_topic').value
        self.passthrough = self.get_parameter('passthrough').value

        # TF for source_frame → target_frame transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to our obstacle boxes (topic switchable via input_topic param)
        self.sub = self.create_subscription(
            MarkerArray,
            self.input_topic,
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
            f'Obstacle Adapter ready — /obstacles/boxes → /obstacle_markers '
            f'(TF: {self.source_frame} → {self.target_frame})'
        )

    def callback(self, msg: MarkerArray):
        self._frame_count += 1

        # Look up source_frame → target_frame transform (2026-07-29: configurable)
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            # 2026-07-29: passthrough mode preserves 3D pipeline classification
            if self.passthrough:
                TYPE_MAP = {'slope': 3, 'bump': 2, 'pole': 1, 'obstacle': 0}
                label = marker.ns
                type_id = TYPE_MAP.get(label, 0)
            else:
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

            # 使用与surface_detector一致的颜色
            r, g, b, a = TYPE_COLORS.get(label, TYPE_COLORS['obstacle'])
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
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
