#!/usr/bin/env python3
"""
Road analyser — extracts boundaries and centreline from the nonground point cloud only.

Subscribes to:
  /patchworkpp/nonground  — obstacles / kerbs / road-edge features

Publishes:
  /lidar/road_boundary_markers  — LINE_STRIP ×2 (left/right road edges, base_link)
  /lidar/centerline             — Path (road midline, base_link, visualisation only)

Algorithm (single-source polar boundary detection):
  1. Polar bin (3° per bin) the nonground cloud.
  2. For each angular bin, take the nearest nonground point as the boundary
     candidate (this is the inner face of the roadside obstacle / kerb).
  3. Reject candidates too close to the vehicle centreline (|y| < min_lateral)
     to keep speed bumps /减速带 out of the boundary set.
  4. Polar-coordinate smoothing per side → publish smooth LINE_STRIP markers.
  5. Temporal caching in world frame keeps road edges visible in the near-field
     LiDAR blind zone.

Key design: does NOT depend on surface_detector or obstacle_adapter classification.
Only consumes raw patchworkpp nonground.  The assumption is that road-edge obstacles
are already classified as nonground by patchworkpp, so the ground cloud is not needed.
"""

import math
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


# ── Polar binning helpers ────────────────────────────────────────────────────

def _bin_points(xyz: np.ndarray, angular_bins: int,
                min_range: float, max_range: float):
    """Polar-bin (N,3) points. Returns arrays indexed by bin.

    Returns:
      bin_points: list of (N_k,3) arrays, one per angular bin
    """
    n = len(xyz)
    if n == 0:
        return [np.zeros((0, 3)) for _ in range(angular_bins)]

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    dist = np.sqrt(x**2 + y**2)
    angle = np.arctan2(y, x)  # [-pi, pi]

    mask = (dist >= min_range) & (dist <= max_range)
    if mask.sum() == 0:
        return [np.zeros((0, 3)) for _ in range(angular_bins)]

    x, y, z, dist, angle = x[mask], y[mask], z[mask], dist[mask], angle[mask]

    bin_edges = np.linspace(-math.pi, math.pi, angular_bins + 1)
    bin_indices = np.digitize(angle, bin_edges) - 1
    bin_indices = np.clip(bin_indices, 0, angular_bins - 1)

    bin_pts = [np.zeros((0, 3)) for _ in range(angular_bins)]
    for bi in range(angular_bins):
        m = bin_indices == bi
        if m.sum() > 0:
            bin_pts[bi] = np.column_stack([x[m], y[m], z[m]])
    return bin_pts


def _nearest_nonground_per_bin(bin_pts: np.ndarray,
                                min_range: float = 1.5):
    """Return the nearest nonground point in the bin beyond min_range.

    This is the inner face of the roadside obstacle / kerb.  Returns
    (x, y, dist) or None.
    """
    n = len(bin_pts)
    if n < 1:
        return None

    dist = np.sqrt(bin_pts[:, 0]**2 + bin_pts[:, 1]**2)
    mask = dist >= min_range
    if mask.sum() == 0:
        return None

    best = np.argmin(dist[mask])
    idx = np.where(mask)[0][best]
    return float(bin_pts[idx, 0]), float(bin_pts[idx, 1]), float(dist[idx])


def _extract_boundaries_nonground(
    nonground_bins: list,
    angular_bins: int,
    min_lateral: float = 0.5,
    min_range: float = 1.5,
) -> tuple:
    """Single-source polar boundary extraction from nonground only.

    For each angular bin the nearest nonground point beyond min_range is taken
    as the boundary candidate.  Points too close to the vehicle centreline
    (|y| < min_lateral) are rejected, which removes mid-road objects (speed
    bumps / 减速带) from the boundary set.

    Returns (left_cands, right_cands) where each is (N,3) with columns
    [angle, x, y] in sensor frame.
    """
    left_cands = []
    right_cands = []

    for bi in range(angular_bins):
        ng = _nearest_nonground_per_bin(nonground_bins[bi], min_range)
        if ng is None:
            continue

        bx, by = ng[0], ng[1]
        # Reject mid-road candidates (speed bumps etc.)
        if abs(by) < min_lateral:
            continue

        angle = math.atan2(by, bx)
        if by > 0:
            left_cands.append([angle, bx, by])
        else:
            right_cands.append([angle, bx, by])

    left_arr = np.array(left_cands) if left_cands else np.zeros((0, 3))
    right_arr = np.array(right_cands) if right_cands else np.zeros((0, 3))
    return left_arr, right_arr


# ── Smoothing ────────────────────────────────────────────────────────────────

def _smooth_boundary_polar(cands: np.ndarray, window: int = 5) -> np.ndarray:
    """Smooth boundary candidates in polar coordinates.

    Input: (N,3) array with columns [angle, x, y].
    Output: (N,2) array of smoothed [x, y] sorted by angle.
    Averaging radius and angle separately preserves radial ordering and
    prevents a single straight line shortcut across the vehicle.
    """
    n = len(cands)
    if n < window:
        return cands[:, 1:] if n > 0 else np.zeros((0, 2))

    arr = cands[np.argsort(cands[:, 0])]
    theta = arr[:, 0]
    r = np.sqrt(arr[:, 1]**2 + arr[:, 2]**2)

    half = window // 2
    r_smooth = np.empty_like(r)
    t_smooth = np.empty_like(theta)

    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        r_smooth[i] = np.mean(r[lo:hi])
        # mean angle handling wrap around the -pi/pi seam
        angs = theta[lo:hi]
        base = angs[len(angs)//2]
        angs_unwrapped = np.mod(angs - base + math.pi, 2 * math.pi) - math.pi + base
        t_smooth[i] = np.mean(angs_unwrapped)

    x = r_smooth * np.cos(t_smooth)
    y = r_smooth * np.sin(t_smooth)
    return np.column_stack([x, y])


# ── Marker building ──────────────────────────────────────────────────────────

def _boundary_to_linestrip(pts: np.ndarray, frame_id: str, ns: str,
                           marker_id: int, now) -> Marker:
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


# ── Centreline ───────────────────────────────────────────────────────────────

def _compute_centerline(left: np.ndarray, right: np.ndarray,
                        frame_id: str, now) -> PathMsg:
    """Compute centreline as midpoint of matched left-right boundary pairs."""
    msg = PathMsg()
    msg.header.frame_id = frame_id
    msg.header.stamp = now

    if len(left) < 3 or len(right) < 3:
        return msg

    ang_l = np.arctan2(left[:, 1], left[:, 0])
    ang_r = np.arctan2(right[:, 1], right[:, 0])
    left_s = left[np.argsort(ang_l)]
    right_s = right[np.argsort(ang_r)]

    ang_l_s = np.arctan2(left_s[:, 1], left_s[:, 0])
    ang_r_s = np.arctan2(right_s[:, 1], right_s[:, 0])

    mid_pts = []
    for i, al in enumerate(ang_l_s):
        j = np.argmin(np.abs(ang_r_s - al))
        if abs(ang_r_s[j] - al) < math.radians(5.0):
            mx = (left_s[i, 0] + right_s[j, 0]) / 2.0
            my = (left_s[i, 1] + right_s[j, 1]) / 2.0
            mid_pts.append([mx, my])

    if len(mid_pts) < 3:
        return msg

    mid_arr = np.array(mid_pts)
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
    """Single-source road boundary extraction from nonground only."""

    def __init__(self):
        super().__init__('road_analyzer')

        # ── Parameters ──
        self.declare_parameter('angular_bins', 120)        # 3°/bin — fewer bins → less zigzag
        self.declare_parameter('min_lateral', 0.5)         # ignore candidates too close to x-axis (mid-road)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('log_interval', 30)
        # 2026-08-06: temporal caching for near-field blind zone persistence
        self.declare_parameter('cache_duration_ms', 2000)   # keep boundaries 2s after last sighting
        self.declare_parameter('world_frame', 'map')        # cache in world frame
        self.declare_parameter('publish_rate_hz', 10.0)     # independent publish rate

        # QoS matching patchworkpp (RELIABLE + TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ──
        # 2026-08-06: only subscribe to nonground; ground cloud causes spurious
        # triangular boundary connections and is not needed when kerbs are nonground.
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
        self._last_centerline = None

        # Temporal cache: boundary points stored as (world_x, world_y, side, stamp_ns)
        self._cache = []   # list of dicts
        self._cache_ns = int(self.get_parameter('cache_duration_ms').value) * 1_000_000
        self._world_frame = self.get_parameter('world_frame').value

        # Latest sensor-frame boundary arrays (set by callbacks, consumed by _publish)
        self._latest_left_sensor = np.zeros((0, 2))
        self._latest_right_sensor = np.zeros((0, 2))
        self._latest_source_frame = ''
        self._latest_nonground_xyz = np.zeros((0, 3))

        self.get_logger().info(
            'Road Analyzer ready — single-source (nonground only) '
            'polar boundary detection + temporal caching')

    # ── TF helpers ──
    def _lookup(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    # ── Callbacks ──
    def _on_nonground(self, msg: PointCloud2):
        self._latest_nonground_xyz = _pc2_to_xyz(msg)
        if not self._latest_source_frame:
            self._latest_source_frame = msg.header.frame_id
        self._try_process()

    def _try_process(self):
        """Process when nonground data is available."""
        if len(self._latest_nonground_xyz) < 10:
            return
        self._process_frame()

    def _process_frame(self):
        self._frame_count += 1

        bins = self.get_parameter('angular_bins').value
        min_r = self.get_parameter('min_range').value
        max_r = self.get_parameter('max_range').value
        win = self.get_parameter('smooth_window').value
        log_int = self.get_parameter('log_interval').value

        # ── Bin nonground only ──
        nonground_bins = _bin_points(self._latest_nonground_xyz, bins, min_r, max_r)

        # ── Nonground-only extraction ──
        min_lat = self.get_parameter('min_lateral').value
        left_cands, right_cands = _extract_boundaries_nonground(
            nonground_bins, bins, min_lat, min_r)

        # smooth in polar coordinates to preserve radial ordering
        left_sm = _smooth_boundary_polar(left_cands, win)
        right_sm = _smooth_boundary_polar(right_cands, win)

        if len(left_sm) < 3 or len(right_sm) < 3:
            if self._frame_count % log_int == 0:
                self.get_logger().warn(
                    f'Frame {self._frame_count}: insufficient boundary points '
                    f'(left={len(left_sm)}, right={len(right_sm)})',
                    throttle_duration_sec=3.0)
            return

        # Store for later publish (with TF transform + temporal caching)
        self._latest_left_sensor = left_sm
        self._latest_right_sensor = right_sm

    # ── Publish ──
    def _publish(self):
        """Transform latest boundaries to base_link, merge with cache, publish."""
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        now_msg = now.to_msg()
        log_int = self.get_parameter('log_interval').value

        source_frame = self._latest_source_frame
        if not source_frame:
            return

        # ── TF: sensor → base_link ──
        t_s2b = self._lookup('base_link', source_frame)
        if t_s2b is None:
            self.get_logger().warn(
                f'TF {source_frame}→base_link unavailable',
                throttle_duration_sec=5.0)
            return

        left_bl = _transform_points(self._latest_left_sensor, t_s2b)
        right_bl = _transform_points(self._latest_right_sensor, t_s2b)

        # ── Update temporal cache (world-frame storage) ──
        t_b2w = self._lookup(self._world_frame, 'base_link')
        if t_b2w is not None and len(left_bl) > 0:
            for px, py in left_bl:
                wx, wy, _ = _apply_transform(t_b2w, px, py, 0.0)
                self._cache.append({
                    'wx': wx, 'wy': wy, 'side': 'left', 'stamp_ns': now_ns})
        if t_b2w is not None and len(right_bl) > 0:
            for px, py in right_bl:
                wx, wy, _ = _apply_transform(t_b2w, px, py, 0.0)
                self._cache.append({
                    'wx': wx, 'wy': wy, 'side': 'right', 'stamp_ns': now_ns})

        # ── Expire old cache entries ──
        self._cache = [e for e in self._cache
                       if now_ns - e['stamp_ns'] <= self._cache_ns]

        # ── Re-project cached world points back to current base_link ──
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

        cached_left = np.array(cached_left) if cached_left else np.zeros((0, 2))
        cached_right = np.array(cached_right) if cached_right else np.zeros((0, 2))

        # Merge live + cached
        if len(left_bl) > 0 and len(cached_left) > 0:
            merged_left = np.vstack([left_bl, cached_left])
        elif len(left_bl) > 0:
            merged_left = left_bl
        else:
            merged_left = cached_left

        if len(right_bl) > 0 and len(cached_right) > 0:
            merged_right = np.vstack([right_bl, cached_right])
        elif len(right_bl) > 0:
            merged_right = right_bl
        else:
            merged_right = cached_right

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
        elif self._last_centerline is not None:
            self._last_centerline.header.stamp = now_msg
            self.pub_centerline.publish(self._last_centerline)

        # ── Log ──
        if self._frame_count % log_int == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'live left={len(left_bl)} right={len(right_bl)}, '
                f'cached left={len(cached_left)} right={len(cached_right)}, '
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
