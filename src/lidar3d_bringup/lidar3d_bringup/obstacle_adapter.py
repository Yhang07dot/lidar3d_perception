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

        # ==== 目标留存 (2026-08-04) ====
        # frenet_planner 是状态覆盖式(self.obstacles = obstacles 整体替换)且每
        # 100ms 独立规划，感知抖动/遮挡时瞬时空帧会让它丢失全部障碍物。
        # 这里缓存障碍物一段时间，并以 world 系存储、发布时反算回 base_link，
        # 使车辆前进时缓存障碍物的相对位置自动正确后退（而非"粘"在原地）。
        self.declare_parameter('obstacle_memory_ms', 500)      # 缓存保留时长(ms)
        self.declare_parameter('enable_dead_reckoning', True)  # 位姿推算(需 world→base_link TF)
        self.declare_parameter('world_frame', 'map')           # 推算参考系
        self.declare_parameter('publish_rate_hz', 20.0)        # 发布频率(独立于感知帧率)
        self.memory_ns = int(self.get_parameter('obstacle_memory_ms').value) * 1_000_000
        self.dead_reckoning = self.get_parameter('enable_dead_reckoning').value
        self.world_frame = self.get_parameter('world_frame').value

        # 缓存: marker id → {world_xyz|base_xyz, scale, label, stamp_ns, has_world}
        self._cache = {}

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

        # 定时发布: 即使感知无输出也维持缓存障碍物的连续发布
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._publish_cached)

        self._frame_count = 0
        self._log_interval = 10

        self.get_logger().info(
            f'Obstacle Adapter ready — {self.input_topic} → /obstacle_markers '
            f'(TF: {self.source_frame} → {self.target_frame}, '
            f'memory={self.memory_ns // 1_000_000}ms, '
            f'dead_reckoning={self.dead_reckoning})'
        )

    def _lookup(self, target: str, source: str):
        """TF lookup helper. Returns TransformStamped or None."""
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    @staticmethod
    def _apply_transform(t: TransformStamped, px: float, py: float, pz: float):
        """Rotate by the transform's quaternion, then translate."""
        tr = t.transform.translation
        q = t.transform.rotation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        # p' = p + 2*cross(q.xyz, cross(q.xyz, p) + q.w*p)
        cx = 2.0 * (qy * pz - qz * py)
        cy = 2.0 * (qz * px - qx * pz)
        cz = 2.0 * (qx * py - qy * px)
        rx = px + qw * cx + (qy * cz - qz * cy)
        ry = py + qw * cy + (qz * cx - qx * cz)
        rz = pz + qw * cz + (qx * cy - qy * cx)
        return rx + tr.x, ry + tr.y, rz + tr.z

    def callback(self, msg: MarkerArray):
        self._frame_count += 1

        # Look up source_frame → target_frame transform (2026-07-29: configurable)
        t = self._lookup(self.target_frame, self.source_frame)
        if t is None:
            self.get_logger().warn('TF lookup failed (sensor→base_link)',
                                   throttle_duration_sec=5.0)
            return

        # world→base_link 的逆变换用于把 base_link 坐标存成 world 坐标
        t_w = self._lookup(self.world_frame, self.target_frame) if self.dead_reckoning else None

        now_ns = self.get_clock().now().nanoseconds

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            # 2026-07-29: passthrough mode preserves 3D pipeline classification
            label = marker.ns if self.passthrough else classify_obstacle(marker.scale)[0]

            # Transform position from sensor frame → base_link
            base_x, base_y, base_z = self._apply_transform(
                t, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

            entry = {
                'scale': marker.scale,
                'label': label,
                'stamp_ns': now_ns,
                'base_xyz': (base_x, base_y, base_z),
                'has_world': False,
            }
            # 入库时记录 world 坐标，发布时可反算回当前 base_link
            if t_w is not None:
                entry['world_xyz'] = self._apply_transform(t_w, base_x, base_y, base_z)
                entry['has_world'] = True

            self._cache[marker.id] = entry

    def _publish_cached(self):
        """Publish cached obstacles, re-projecting world coords into current base_link."""
        now_ns = self.get_clock().now().nanoseconds

        # 清理过期条目
        expired = [k for k, v in self._cache.items()
                   if now_ns - v['stamp_ns'] > self.memory_ns]
        for k in expired:
            del self._cache[k]

        if not self._cache:
            self.pub.publish(MarkerArray())
            return

        # base_link←world 变换：把缓存的 world 坐标反算回当前车体系
        t_b = self._lookup(self.target_frame, self.world_frame) if self.dead_reckoning else None
        if self.dead_reckoning and t_b is None:
            self.get_logger().warn(
                f'TF {self.world_frame}→{self.target_frame} unavailable — '
                'holding cached positions without dead reckoning',
                throttle_duration_sec=5.0)

        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for mid, e in self._cache.items():
            if t_b is not None and e['has_world']:
                wx, wy, wz = e['world_xyz']
                px, py, pz = self._apply_transform(t_b, wx, wy, wz)
            else:
                px, py, pz = e['base_xyz']  # 降级: 保持入库时的相对位置

            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = now
            m.ns = e['label']
            m.id = mid
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            m.pose.orientation.w = 1.0
            m.scale = e['scale']

            # 使用与surface_detector一致的颜色
            r, g, b, a = TYPE_COLORS.get(e['label'], TYPE_COLORS['obstacle'])
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            m.lifetime.nanosec = 200_000_000  # 200ms
            out.markers.append(m)

        self.pub.publish(out)

        if self._frame_count % self._log_interval == 0 and out.markers:
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
