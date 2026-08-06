#!/usr/bin/env python3
"""
Road analyser — extracts lane-like road boundaries from nonground obstacles.

Subscribes to:
  /patchworkpp/nonground  — obstacles / kerbs / road-edge features

Publishes:
  /lidar/road_boundary_markers  — LINE_STRIP ×2 (left/right road edges, base_link)
  /lidar/centerline             — Path (road midline, base_link, visualisation only)

Algorithm (single-source longitudinal lane tracking):
  1. Transform nonground points into base_link and keep only the forward field.
  2. Divide the road ahead into longitudinal (x-axis) bins.
  3. In each bin, take the inner faces of the left and right roadside obstacles.
  4. Retain width-consistent obstacle pairs and reject only abrupt local jumps.
  5. Smooth and publish two x-ordered LINE_STRIP markers, one per road side.
  6. Temporal caching in world frame keeps road edges visible in the near-field
     LiDAR blind zone without reordering the two lane lines.

Key design: does NOT depend on surface_detector or obstacle_adapter classification.
Only consumes raw patchworkpp nonground.  The assumption is that road-edge obstacles
are already classified as nonground by patchworkpp, so the ground cloud is not needed.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA


# ── PointCloud2 → numpy ──────────────────────────────────────────────────────

DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Decode PointCloud2 into (N,3) float array [x, y, z]."""
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({
        'names': names, 'formats': formats,
        'offsets': offsets, 'itemsize': msg.point_step,
    })
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


# ── Longitudinal lane tracking helpers ───────────────────────────────────────

def _empty_points() -> np.ndarray:
    """Return an empty XY point array with a stable shape."""
    return np.zeros((0, 2))


def _smooth_lane_track(points: np.ndarray, window: int) -> np.ndarray:
    """Median-smooth an x-ordered lane track without changing its x values."""
    if len(points) == 0:
        return _empty_points()

    ordered = points[np.argsort(points[:, 0])]
    if len(ordered) < window or window <= 1:
        return ordered

    half = window // 2
    smoothed = ordered.copy()
    for index in range(len(ordered)):
        start = max(0, index - half)
        stop = min(len(ordered), index + half + 1)
        smoothed[index, 1] = np.median(ordered[start:stop, 1])
    return smoothed


def _filter_pair_outliers(
    left: np.ndarray,
    right: np.ndarray,
    forward_bin_size: float,
    max_x_gap: float,
    max_lateral_step: float,
) -> tuple:
    """Keep sparse pair samples while rejecting abrupt local lateral jumps."""
    if len(left) < 2 or len(right) < 2:
        return _empty_points(), _empty_points()

    order = np.argsort(left[:, 0])
    left = left[order]
    right = right[order]

    accepted = [0]
    for index in range(1, len(left)):
        previous = accepted[-1]
        x_gap = left[index, 0] - left[previous, 0]
        lateral_scale = max(1.0, x_gap / forward_bin_size)
        allowed_step = max_lateral_step * lateral_scale
        left_step = abs(left[index, 1] - left[previous, 1])
        right_step = abs(right[index, 1] - right[previous, 1])
        if x_gap > max_x_gap and lateral_scale > 1.0:
            accepted.append(index)
        elif left_step <= allowed_step and right_step <= allowed_step:
            accepted.append(index)

    return left[accepted], right[accepted]


def _extract_lane_boundaries(
    xyz: np.ndarray,
    min_forward: float,
    max_forward: float,
    min_lateral: float,
    forward_bin_size: float,
    min_road_width: float,
    max_road_width: float,
    width_tolerance: float,
    max_lateral_step: float,
    min_points_per_side: int,
) -> tuple:
    """
    Extract paired left/right road-edge candidates from obstacle points.

    The vehicle frame convention is x forward, y left.  A candidate pair is
    made only when obstacle points exist on both sides of one longitudinal bin.
    Selecting the innermost obstacle face makes kerbs, barriers, and obstacle
    walls form the lane boundary while the paired-width test rejects isolated
    objects inside the drivable corridor.
    """
    if len(xyz) == 0:
        return _empty_points(), _empty_points()

    x = xyz[:, 0]
    y = xyz[:, 1]
    finite = np.isfinite(x) & np.isfinite(y)
    radial_range = np.hypot(x, y)
    mask = (
        finite &
        (x >= min_forward) &
        (x <= max_forward) &
        (radial_range <= max_forward)
    )
    if not np.any(mask):
        return _empty_points(), _empty_points()

    x = x[mask]
    y = y[mask]
    bin_indices = np.floor((x - min_forward) / forward_bin_size).astype(int)

    left_candidates = []
    right_candidates = []
    for bin_index in np.unique(bin_indices):
        bin_mask = bin_indices == bin_index
        x_values = x[bin_mask]
        y_values = y[bin_mask]
        left_values = y_values[y_values >= min_lateral]
        right_values = y_values[y_values <= -min_lateral]
        if (len(left_values) < min_points_per_side or
                len(right_values) < min_points_per_side):
            continue

        left_y = float(np.quantile(left_values, 0.10))
        right_y = float(np.quantile(right_values, 0.90))
        road_width = left_y - right_y
        if not min_road_width <= road_width <= max_road_width:
            continue

        x_position = float(np.median(x_values))
        left_candidates.append([x_position, left_y])
        right_candidates.append([x_position, right_y])

    if len(left_candidates) < 2:
        return _empty_points(), _empty_points()

    left = np.asarray(left_candidates)
    right = np.asarray(right_candidates)
    widths = left[:, 1] - right[:, 1]
    median_width = float(np.median(widths))
    width_mask = np.abs(widths - median_width) <= width_tolerance
    left = left[width_mask]
    right = right[width_mask]
    return _filter_pair_outliers(
        left,
        right,
        forward_bin_size=forward_bin_size,
        max_x_gap=forward_bin_size * 2.5,
        max_lateral_step=max_lateral_step,
    )


def _merge_live_with_cache(live: np.ndarray, cached: np.ndarray,
                           forward_bin_size: float,
                           smooth_window: int) -> np.ndarray:
    """Use live boundary samples first and fill only missing bins from cache."""
    if len(live) == 0 and len(cached) == 0:
        return _empty_points()

    def bin_medians(track: np.ndarray) -> dict:
        if len(track) == 0:
            return {}
        indices = np.floor(track[:, 0] / forward_bin_size).astype(int)
        return {
            index: np.median(track[indices == index], axis=0)
            for index in np.unique(indices)
        }

    live_bins = bin_medians(live)
    cached_bins = bin_medians(cached)
    track = [
        live_bins[index] if index in live_bins else cached_bins[index]
        for index in sorted(set(live_bins) | set(cached_bins))
    ]
    return _smooth_lane_track(np.asarray(track), smooth_window)


def _median_nearest_distance(points: np.ndarray,
                              reference: np.ndarray) -> float:
    """Return a map-frame overlap consistency metric for one boundary side."""
    if len(points) == 0 or len(reference) == 0:
        return 0.0
    deltas = points[:, np.newaxis, :] - reference[np.newaxis, :, :]
    distances = np.hypot(deltas[:, :, 0], deltas[:, :, 1])
    return float(np.median(np.min(distances, axis=1)))


# ── Marker building ──────────────────────────────────────────────────────────

def _boundary_to_linestrip(pts: np.ndarray, frame_id: str, ns: str,
                           marker_id: int, now) -> Marker:
    """Build a forward-ordered LINE_STRIP marker from boundary points."""
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

    for p in pts[np.argsort(pts[:, 0])]:
        from geometry_msgs.msg import Point
        pt = Point()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = 0.08
        marker.points.append(pt)
    return marker


# ── Centreline ───────────────────────────────────────────────────────────────

def _compute_centerline(left: np.ndarray, right: np.ndarray,
                        frame_id: str, now) -> PathMsg:
    """Compute the centreline from left/right boundaries at common x values."""
    msg = PathMsg()
    msg.header.frame_id = frame_id
    msg.header.stamp = now

    if len(left) < 3 or len(right) < 3:
        return msg

    left_s = left[np.argsort(left[:, 0])]
    right_s = right[np.argsort(right[:, 0])]
    start_x = max(left_s[0, 0], right_s[0, 0])
    stop_x = min(left_s[-1, 0], right_s[-1, 0])
    if stop_x <= start_x:
        return msg

    count = min(len(left_s), len(right_s))
    if count < 3:
        return msg
    x_values = np.linspace(start_x, stop_x, count)
    left_y = np.interp(x_values, left_s[:, 0], left_s[:, 1])
    right_y = np.interp(x_values, right_s[:, 0], right_s[:, 1])

    for x_value, left_value, right_value in zip(x_values, left_y, right_y):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(x_value)
        ps.pose.position.y = float((left_value + right_value) / 2.0)
        ps.pose.position.z = 0.08
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)

    return msg


# ── TF helpers ───────────────────────────────────────────────────────────────

def _apply_transform(t: TransformStamped, px: float, py: float, pz: float):
    """Rotate by quaternion then translate."""
    tr = t.transform.translation
    q = t.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    cx = 2.0 * (qy * pz - qz * py)
    cy = 2.0 * (qz * px - qx * pz)
    cz = 2.0 * (qx * py - qy * px)
    rx = px + qw * cx + (qy * cz - qz * cy)
    ry = py + qw * cy + (qz * cx - qx * cz)
    rz = pz + qw * cz + (qx * cy - qy * cx)
    return rx + tr.x, ry + tr.y, rz + tr.z


def _transform_points(pts: np.ndarray, transform: TransformStamped) -> np.ndarray:
    """Apply TF transform to (N,2) XY points."""
    if len(pts) == 0:
        return pts
    out = np.zeros_like(pts)
    tr = transform.transform.translation
    q = transform.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    for i, (px, py) in enumerate(pts):
        cx = 2.0 * (qy * 0.0 - qz * py)
        cy = 2.0 * (qz * px - qx * 0.0)
        cz = 2.0 * (qx * py - qy * px)
        rx = px + qw * cx + (qy * cz - qz * cy)
        ry = py + qw * cy + (qz * cx - qx * cz)
        out[i, 0] = rx + tr.x
        out[i, 1] = ry + tr.y
    return out


# ── Node ─────────────────────────────────────────────────────────────────────

class RoadAnalyzer(Node):
    """Generate paired, forward-facing road boundaries from nonground points."""

    def __init__(self):
        super().__init__('road_analyzer')

        # ── Parameters ──
        self.declare_parameter('min_forward', 1.0)
        self.declare_parameter('max_forward', 30.0)
        self.declare_parameter('min_lateral', 0.75)
        self.declare_parameter('forward_bin_size', 0.5)
        self.declare_parameter('min_road_width', 3.0)
        self.declare_parameter('max_road_width', 12.0)
        self.declare_parameter('road_width_tolerance', 1.5)
        self.declare_parameter('max_lateral_step', 1.5)
        self.declare_parameter('min_points_per_side', 2)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('log_interval', 30)
        self.declare_parameter('cache_duration_ms', 2000)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('world_continuity_threshold_m', 0.8)
        self.declare_parameter('publish_rate_hz', 10.0)

        # QoS matching patchworkpp (RELIABLE + TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_nonground = self.create_subscription(
            PointCloud2, '/patchworkpp/nonground', self._on_nonground, qos)

        # ── Publishers ──
        self.pub_boundaries = self.create_publisher(
            MarkerArray, '/lidar/road_boundary_markers', 10)
        self.pub_centerline = self.create_publisher(
            PathMsg, '/lidar/centerline', latched)

        # ── Timed publish (independent of input frame rate) ──
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        # ── State ──
        self._frame_count = 0
        self._last_logged_frame = -1
        self._last_centerline = None

        self._cache = []
        self._cache_ns = int(self.get_parameter('cache_duration_ms').value) * 1_000_000
        self._world_frame = self.get_parameter('world_frame').value
        self._world_continuity_threshold = float(
            self.get_parameter('world_continuity_threshold_m').value)

        self._latest_left_base = _empty_points()
        self._latest_right_base = _empty_points()
        self._latest_source_frame = ''
        self._latest_nonground_xyz = np.zeros((0, 3))
        self._latest_frame_sequence = 0
        self._last_cached_frame_sequence = -1
        self._latest_live_accepted = False

        self.get_logger().info(
            'Road Analyzer ready — live-priority lane tracking '
            '+ map-frame cache fill')

    # ── TF helpers ──
    def _lookup(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    # ── Callbacks ──
    def _on_nonground(self, msg: PointCloud2):
        self._latest_frame_sequence += 1
        self._latest_nonground_xyz = _pc2_to_xyz(msg)
        self._latest_source_frame = msg.header.frame_id
        self._latest_left_base = _empty_points()
        self._latest_right_base = _empty_points()
        self._try_process()

    def _try_process(self):
        """Process when nonground data is available."""
        if len(self._latest_nonground_xyz) < 10:
            return
        self._process_frame()

    def _process_frame(self):
        self._frame_count += 1

        min_forward = self.get_parameter('min_forward').value
        max_forward = self.get_parameter('max_forward').value
        min_lateral = self.get_parameter('min_lateral').value
        bin_size = self.get_parameter('forward_bin_size').value
        min_width = self.get_parameter('min_road_width').value
        max_width = self.get_parameter('max_road_width').value
        width_tolerance = self.get_parameter('road_width_tolerance').value
        max_lateral_step = self.get_parameter('max_lateral_step').value
        min_points = self.get_parameter('min_points_per_side').value
        win = self.get_parameter('smooth_window').value
        log_int = self.get_parameter('log_interval').value

        transform = self._lookup('base_link', self._latest_source_frame)
        if transform is None:
            self.get_logger().warn(
                f'TF {self._latest_source_frame}→base_link unavailable',
                throttle_duration_sec=5.0)
            return

        base_xy = _transform_points(self._latest_nonground_xyz[:, :2], transform)
        base_xyz = np.column_stack([base_xy, self._latest_nonground_xyz[:, 2]])
        left, right = _extract_lane_boundaries(
            base_xyz,
            min_forward,
            max_forward,
            min_lateral,
            bin_size,
            min_width,
            max_width,
            width_tolerance,
            max_lateral_step,
            min_points,
        )
        left = _smooth_lane_track(left, win)
        right = _smooth_lane_track(right, win)

        if len(left) < 3 or len(right) < 3:
            if self._frame_count % log_int == 0:
                self.get_logger().warn(
                    f'Frame {self._frame_count}: insufficient lane pairs '
                    f'(left={len(left)}, right={len(right)})',
                    throttle_duration_sec=3.0)
            return

        self._latest_left_base = left
        self._latest_right_base = right

    # ── Publish ──
    def _publish(self):
        """Publish accepted live boundaries with map-cache hole filling."""
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        now_msg = now.to_msg()
        log_int = self.get_parameter('log_interval').value
        bin_size = self.get_parameter('forward_bin_size').value
        smooth_window = self.get_parameter('smooth_window').value
        min_forward = self.get_parameter('min_forward').value
        max_forward = self.get_parameter('max_forward').value

        left_live = self._latest_left_base
        right_live = self._latest_right_base
        is_new_frame = (
            self._latest_frame_sequence != self._last_cached_frame_sequence
        )
        self._cache = [e for e in self._cache
                       if now_ns - e['stamp_ns'] <= self._cache_ns]

        cached_left_world = np.asarray([
            [entry['wx'], entry['wy']]
            for entry in self._cache if entry['side'] == 'left'
        ])
        cached_right_world = np.asarray([
            [entry['wx'], entry['wy']
            ] for entry in self._cache if entry['side'] == 'right'
        ])
        if len(cached_left_world) == 0:
            cached_left_world = _empty_points()
        if len(cached_right_world) == 0:
            cached_right_world = _empty_points()

        if is_new_frame:
            self._latest_live_accepted = False
            if len(left_live) > 0 and len(right_live) > 0:
                transform = self._lookup(self._world_frame, 'base_link')
                if transform is not None:
                    left_world = _transform_points(left_live, transform)
                    right_world = _transform_points(right_live, transform)
                    left_jump = _median_nearest_distance(
                        left_world, cached_left_world)
                    right_jump = _median_nearest_distance(
                        right_world, cached_right_world)
                    left_valid = (
                        len(cached_left_world) < 3
                        or left_jump <= self._world_continuity_threshold
                    )
                    right_valid = (
                        len(cached_right_world) < 3
                        or right_jump <= self._world_continuity_threshold
                    )
                    self._latest_live_accepted = left_valid and right_valid
                    if self._latest_live_accepted:
                        for side, track in (('left', left_world), ('right', right_world)):
                            for wx, wy in track:
                                self._cache.append({
                                    'wx': wx,
                                    'wy': wy,
                                    'side': side,
                                    'stamp_ns': now_ns,
                                })
                    elif self._frame_count % log_int == 0:
                        self.get_logger().warn(
                            f'Frame {self._frame_count}: rejected live boundaries '
                            f'(map jump left={left_jump:.2f}m right={right_jump:.2f}m)',
                            throttle_duration_sec=2.0)
            self._last_cached_frame_sequence = self._latest_frame_sequence

        if not self._latest_live_accepted:
            left_live = _empty_points()
            right_live = _empty_points()

        t_w2b = self._lookup('base_link', self._world_frame)
        cached_left = []
        cached_right = []
        if t_w2b is not None:
            for e in self._cache:
                bx, by, _ = _apply_transform(t_w2b, e['wx'], e['wy'], 0.0)
                if e['side'] == 'left':
                    cached_left.append([bx, by])
                else:
                    cached_right.append([bx, by])

        cached_left = np.asarray(cached_left) if cached_left else _empty_points()
        cached_right = np.asarray(cached_right) if cached_right else _empty_points()
        if len(cached_left) > 0:
            cached_left = cached_left[
                (cached_left[:, 0] >= min_forward) &
                (cached_left[:, 0] <= max_forward)
            ]
        if len(cached_right) > 0:
            cached_right = cached_right[
                (cached_right[:, 0] >= min_forward) &
                (cached_right[:, 0] <= max_forward)
            ]

        merged_left = _merge_live_with_cache(
            left_live, cached_left, bin_size, smooth_window)
        merged_right = _merge_live_with_cache(
            right_live, cached_right, bin_size, smooth_window)

        # ── Publish boundary markers ──
        markers = MarkerArray()
        markers.markers.append(
            _boundary_to_linestrip(merged_left, 'base_link', 'road_left', 0, now_msg))
        markers.markers.append(
            _boundary_to_linestrip(merged_right, 'base_link', 'road_right', 1, now_msg))
        self.pub_boundaries.publish(markers)

        # ── Publish centreline ──
        centerline = _compute_centerline(merged_left, merged_right, 'base_link', now_msg)
        if len(centerline.poses) >= 3:
            self._last_centerline = centerline
            self.pub_centerline.publish(centerline)
        else:
            self._last_centerline = None
            self.pub_centerline.publish(centerline)

        if (self._frame_count > 0 and
                self._frame_count % log_int == 0 and
                self._last_logged_frame != self._frame_count):
            self._last_logged_frame = self._frame_count
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'live left={len(left_live)} right={len(right_live)}, '
                f'cached left={len(cached_left)} right={len(cached_right)}, '
                f'live_accepted={self._latest_live_accepted}, '
                f'centreline={len(centerline.poses)}pts')


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
