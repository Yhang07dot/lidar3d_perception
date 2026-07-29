#!/usr/bin/env python3
"""
Road analyser — extracts boundaries and centreline from ground point cloud.

Subscribes to /patchworkpp/ground, publishes:
  /road_boundary_markers  — LINE_STRIP ×2 (left/right road edges, base_link frame)
  /lidar/centerline        — Path (road midline, base_link frame, visualisation only)

Algorithm: polar binning → gap detection → boundary points → centreline.
TF lookup transforms boundary points from sensor frame → base_link.

Added 2026-07-29: LiDAR-based road perception.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros.transformations import quaternion_multiply, quaternion_from_euler
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA


DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({
        'names': names, 'formats': formats,
        'offsets': offsets, 'itemsize': msg.point_step,
    })
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


def _extract_boundaries(
    xyz: np.ndarray,
    angular_bins: int = 360,
    gap_threshold: float = 0.8,
    min_range: float = 0.5,
    max_range: float = 30.0,
) -> tuple:
    """Extract left and right boundary points from ground cloud via polar gap detection.

    Returns (left_pts, right_pts) as (N,2) arrays in XY (sensor frame).
    """
    n = len(xyz)
    if n < 10:
        return np.zeros((0, 2)), np.zeros((0, 2))

    x, y = xyz[:, 0], xyz[:, 1]
    dist = np.sqrt(x**2 + y**2)
    angle = np.arctan2(y, x)  # [-pi, pi]

    # filter range
    mask = (dist >= min_range) & (dist <= max_range)
    x, y, dist, angle = x[mask], y[mask], dist[mask], angle[mask]

    # bin by angle
    bin_edges = np.linspace(-math.pi, math.pi, angular_bins + 1)
    bin_indices = np.digitize(angle, bin_edges) - 1
    bin_indices = np.clip(bin_indices, 0, angular_bins - 1)

    left_pts = []
    right_pts = []

    for bi in range(angular_bins):
        mask_bin = bin_indices == bi
        if mask_bin.sum() < 2:
            continue

        # sort by distance
        idx_sorted = np.argsort(dist[mask_bin])
        d_sorted = dist[mask_bin][idx_sorted]
        x_sorted = x[mask_bin][idx_sorted]
        y_sorted = y[mask_bin][idx_sorted]

        # find largest continuous segment (gap > threshold between consecutive points)
        gaps = np.diff(d_sorted)
        break_points = np.where(gaps > gap_threshold)[0]

        if len(break_points) == 0:
            # all points continuous → farthest point is boundary
            edge_idx = -1
        else:
            # first segment ends at first break
            edge_idx = break_points[0]

        if edge_idx >= 0:
            bx, by = x_sorted[edge_idx], y_sorted[edge_idx]
            if by > 0:
                left_pts.append([bx, by])
            else:
                right_pts.append([bx, by])

    left_arr = np.array(left_pts) if left_pts else np.zeros((0, 2))
    right_arr = np.array(right_pts) if right_pts else np.zeros((0, 2))

    return left_arr, right_arr


def _smooth_boundary(pts: np.ndarray, window: int = 5) -> np.ndarray:
    """Moving-average smooth boundary points (sorted by angle)."""
    if len(pts) < window:
        return pts
    # sort by angle around origin
    angles = np.arctan2(pts[:, 1], pts[:, 0])
    order = np.argsort(angles)
    pts_sorted = pts[order]
    # moving average
    smoothed = np.zeros_like(pts_sorted)
    half = window // 2
    for i in range(len(pts_sorted)):
        lo = max(0, i - half)
        hi = min(len(pts_sorted), i + half + 1)
        smoothed[i] = pts_sorted[lo:hi].mean(axis=0)
    return smoothed


def _boundary_to_linestrip(pts: np.ndarray, frame_id: str, ns: str, marker_id: int, now) -> Marker:
    """Build a LINE_STRIP marker from boundary points."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = now
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.09  # line width
    marker.color = ColorRGBA(r=0.1, g=0.85, b=1.0, a=1.0)  # cyan
    marker.lifetime.nanosec = 200_000_000

    for p in pts:
        from geometry_msgs.msg import Point
        pt = Point()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = 0.08
        marker.points.append(pt)

    return marker


def _compute_centerline(left: np.ndarray, right: np.ndarray, frame_id: str, now) -> PathMsg:
    """Compute centreline as midpoint of matched left-right boundary pairs."""
    msg = PathMsg()
    msg.header.frame_id = frame_id
    msg.header.stamp = now

    if len(left) < 3 or len(right) < 3:
        return msg

    # sort both by angle
    ang_l = np.arctan2(left[:, 1], left[:, 0])
    ang_r = np.arctan2(right[:, 1], right[:, 0])
    left_s = left[np.argsort(ang_l)]
    right_s = right[np.argsort(ang_r)]

    # match by nearest angular neighbour
    ang_l_s = np.arctan2(left_s[:, 1], left_s[:, 0])
    ang_r_s = np.arctan2(right_s[:, 1], right_s[:, 0])

    # for each left point, find closest-angle right point
    mid_pts = []
    for i, al in enumerate(ang_l_s):
        j = np.argmin(np.abs(ang_r_s - al))
        if abs(ang_r_s[j] - al) < math.radians(5.0):  # 5° max angular gap
            mx = (left_s[i, 0] + right_s[j, 0]) / 2.0
            my = (left_s[i, 1] + right_s[j, 1]) / 2.0
            mid_pts.append([mx, my])

    if len(mid_pts) < 3:
        return msg

    mid_arr = np.array(mid_pts)
    # sort by distance from origin
    dists = np.sqrt(mid_arr[:, 0]**2 + mid_arr[:, 1]**2)
    mid_sorted = mid_arr[np.argsort(dists)]

    for p in mid_sorted:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(p[0])
        ps.pose.position.y = float(p[1])
        ps.pose.position.z = 0.08
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)

    return msg


def _transform_points(pts: np.ndarray, transform) -> np.ndarray:
    """Apply TF transform (translation + quaternion rotation) to (N,2) XY points."""
    if len(pts) == 0:
        return pts
    tr = transform.transform.translation
    q = transform.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z

    # rotate points by quaternion (only XY rotation matters for 2D boundary)
    out = np.zeros_like(pts)
    for i, (px, py) in enumerate(pts):
        # quaternion rotate (px, py, 0)
        cx = 2.0 * (qy * 0.0 - qz * py)
        cy = 2.0 * (qz * px - qx * 0.0)
        cz = 2.0 * (qx * py - qy * px)
        rx = px + qw * cx + (qy * cz - qz * cy)
        ry = py + qw * cy + (qz * cx - qx * cz)
        out[i, 0] = rx + tr.x
        out[i, 1] = ry + tr.y
    return out


class RoadAnalyzer(Node):
    """Extract road boundaries and centreline from ground point cloud."""

    def __init__(self):
        super().__init__('road_analyzer')

        self.declare_parameter('angular_bins', 360)
        self.declare_parameter('gap_threshold', 0.8)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('log_interval', 30)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # TF for sensor_frame → base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, '/patchworkpp/ground', self._callback, qos,
        )
        self.pub_boundaries = self.create_publisher(
            MarkerArray, '/road_boundary_markers', 10,
        )
        self.pub_centerline = self.create_publisher(
            PathMsg, '/lidar/centerline', latched,  # visualisation only (truth provides /reference_centerline)
        )

        self._frame_count = 0
        self._last_centerline = None

        self.get_logger().info('Road Analyzer ready — TF to base_link + polar gap boundary')

    def _callback(self, msg: PointCloud2):
        self._frame_count += 1
        xyz = _pc2_to_xyz(msg)

        if len(xyz) < 10:
            return

        bins = self.get_parameter('angular_bins').value
        gap = self.get_parameter('gap_threshold').value
        min_r = self.get_parameter('min_range').value
        max_r = self.get_parameter('max_range').value
        win = self.get_parameter('smooth_window').value
        log_int = self.get_parameter('log_interval').value

        left, right = _extract_boundaries(xyz, bins, gap, min_r, max_r)

        # smooth
        left_sm = _smooth_boundary(left, win)
        right_sm = _smooth_boundary(right, win)

        # --- TF: transform boundary points sensor_frame → base_link ---
        source_frame = msg.header.frame_id
        try:
            t = self.tf_buffer.lookup_transform('base_link', source_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup {source_frame}→base_link failed: {e}', throttle_duration_sec=3.0)
            return

        left_bl = _transform_points(left_sm, t)
        right_bl = _transform_points(right_sm, t)

        now = self.get_clock().now().to_msg()
        pub_frame = 'base_link'

        # --- publish boundaries (base_link frame) ---
        markers = MarkerArray()
        markers.markers.append(_boundary_to_linestrip(left_bl, pub_frame, 'road_left', 0, now))
        markers.markers.append(_boundary_to_linestrip(right_bl, pub_frame, 'road_right', 1, now))
        self.pub_boundaries.publish(markers)

        # --- publish centreline (visualisation only, base_link frame) ---
        centerline = _compute_centerline(left_bl, right_bl, pub_frame, now)
        if len(centerline.poses) >= 3:
            self._last_centerline = centerline
            self.pub_centerline.publish(centerline)
        elif self._last_centerline is not None:
            # republish last good centreline
            self._last_centerline.header.stamp = now
            self.pub_centerline.publish(self._last_centerline)

        if self._frame_count % log_int == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'left={len(left_sm)}pts right={len(right_sm)}pts '
                f'centreline={len(centerline.poses)}pts'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RoadAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
