#!/usr/bin/env python3
"""
Voxel-based obstacle analyser — replaces PCA-on-raw-clusters with grid-level geometric features.

Subscribes to /patchworkpp/nonground, publishes:
  /obstacles/boxes_3d_voxel   — CUBE MarkerArray (high-confidence, 4-class)
  /lidar/low_confidence_voxel  — CUBE MarkerArray (low-confidence, debug only)

Algorithm:
  1. multi-resolution 3D voxel grid (0-15m:0.1m, 15-30m:0.2m, 30-50m:0.4m)
  2. per-voxel outlier removal (top 5% floating points)
  3. per-voxel geometric features (z_range, z_variance, density)
  4. 26-neighbour voxel flood-fill → objects
  5. per-object aggregated features → rule classification
  6. temporal tracking + confidence scoring

Tuning guide:
  voxel_size_{near,mid,far}: smaller=sharper boundaries, larger=more stable stats
  outlier_pct: higher=more aggressive noise removal
  min_total_points: lower=detect smaller objects (but more false positives)

Added 2026-07-30: voxel-based geometric analysis replacing PCA-on-clusters.
"""

import math
from collections import Counter, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# --- constants (shared with cluster_analyzer) ---
DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}
TYPE_OBSTACLE, TYPE_POLE, TYPE_BUMP, TYPE_SLOPE = 0, 1, 2, 3
TYPE_LABELS = {0: 'obstacle', 1: 'pole', 2: 'bump', 3: 'slope'}
TYPE_COLORS = {
    0: ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6),
    1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),
    2: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5),
    3: ColorRGBA(r=0.0, g=0.5, b=0.0, a=0.45),
}


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({'names': names, 'formats': formats,
                       'offsets': offsets, 'itemsize': msg.point_step})
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


# ---------------------------------------------------------------------------
# voxel grid building
# ---------------------------------------------------------------------------

def _build_multires_grid(xyz: np.ndarray) -> dict:
    """Build multi-resolution 3D voxel grid.

    Returns dict: {(ix, iy, iz): {'pts': [indices], 'voxel_size': float}}
    Zones: near(0-15m,0.1m), mid(15-30m,0.2m), far(30-50m,0.4m).
    """
    dist = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2)
    grid = {}

    zones = [
        (0.0, 15.0, 0.10),
        (15.0, 30.0, 0.20),
        (30.0, 50.0, 0.40),
    ]

    for z_min, z_max, vs in zones:
        mask = (dist >= z_min) & (dist < z_max)
        if not mask.any():
            continue
        pts_zone = xyz[mask]
        indices_zone = np.where(mask)[0]
        voxel_indices = np.floor(pts_zone / vs).astype(np.int32)

        for vi in range(len(pts_zone)):
            key = (voxel_indices[vi, 0], voxel_indices[vi, 1], voxel_indices[vi, 2])
            if key not in grid:
                grid[key] = {'pts': [], 'voxel_size': vs}
            grid[key]['pts'].append(indices_zone[vi])

    return grid


# ---------------------------------------------------------------------------
# per-voxel outlier removal
# ---------------------------------------------------------------------------

def _remove_outliers(grid: dict, xyz: np.ndarray, pct: float = 0.05) -> dict:
    """Remove top `pct` highest-Z points from each voxel (floating noise).

    Returns filtered grid dict (same structure, reduced point lists).
    """
    for key, cell in grid.items():
        indices = cell['pts']
        if len(indices) < 5:
            continue  # too few points to filter meaningfully
        z_vals = xyz[indices, 2]
        cutoff = np.percentile(z_vals, 100 * (1.0 - pct))
        cell['pts'] = [i for i, z in zip(indices, z_vals) if z <= cutoff]
    return grid


# ---------------------------------------------------------------------------
# per-voxel feature extraction
# ---------------------------------------------------------------------------

def _voxel_features(grid: dict, xyz: np.ndarray) -> dict:
    """Compute per-voxel geometric features.

    Returns dict with same keys, adding: centroid, z_range, z_variance, n_pts, density.
    Removes voxels with < min_pts (default 3).
    """
    result = {}
    for key, cell in grid.items():
        indices = cell['pts']
        if len(indices) < 3:
            continue
        pts = xyz[indices]
        cell['centroid'] = pts.mean(axis=0)
        cell['z_range'] = float(pts[:, 2].max() - pts[:, 2].min())
        cell['z_variance'] = float(pts[:, 2].var()) if len(indices) > 1 else 0.0
        cell['n_pts'] = len(indices)
        cell['density'] = len(indices) / (cell['voxel_size'] ** 3)
        result[key] = cell
    return result


# ---------------------------------------------------------------------------
# voxel clustering (26-neighbour flood fill)
# ---------------------------------------------------------------------------

def _cluster_voxels(features: dict) -> list:
    """Flood-fill connected occupied voxels → list of object dicts.

    Each object dict: {voxel_keys, all_point_indices, centroid, bbox, total_n}.
    """
    # build occupancy set
    occupied = set(features.keys())
    visited = set()
    objects = []

    offsets = np.array([
        [dx, dy, dz]
        for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ], dtype=np.int32)

    for seed in occupied:
        if seed in visited:
            continue
        visited.add(seed)
        frontier = [seed]
        obj_voxels = [seed]

        while frontier:
            current = frontier.pop()
            for off in offsets:
                nbr = (current[0] + off[0], current[1] + off[1], current[2] + off[2])
                if nbr in occupied and nbr not in visited:
                    visited.add(nbr)
                    frontier.append(nbr)
                    obj_voxels.append(nbr)

        # aggregate
        all_pts = []
        centroids = []
        for vk in obj_voxels:
            cell = features[vk]
            all_pts.extend(cell['pts'])
            centroids.append(cell['centroid'])

        centroids_arr = np.array(centroids)
        objects.append({
            'voxel_keys': obj_voxels,
            'all_point_indices': all_pts,
            'total_n': len(all_pts),
            'n_voxels': len(obj_voxels),
            'centroids': centroids_arr,
            'mean_centroid': centroids_arr.mean(axis=0),
        })

    return objects


# ---------------------------------------------------------------------------
# object-level feature aggregation + classification
# ---------------------------------------------------------------------------

def _object_features(obj: dict, features: dict, xyz: np.ndarray) -> dict:
    """Compute aggregated features for one object."""
    z_ranges = [features[vk]['z_range'] for vk in obj['voxel_keys']]
    z_vars = [features[vk]['z_variance'] for vk in obj['voxel_keys']]

    indices = obj['all_point_indices']
    pts = xyz[indices]

    min_pt = pts.min(axis=0)
    max_pt = pts.max(axis=0)
    dims = max_pt - min_pt

    return {
        'dims': dims,
        'centroid': (min_pt + max_pt) / 2.0,
        'total_n': obj['total_n'],
        'n_voxels': obj['n_voxels'],
        'mean_z_range': float(np.mean(z_ranges)),
        'mean_z_variance': float(np.mean(z_vars)),
        'occupancy': obj['n_voxels'] / max(1, np.prod(np.ceil(dims / 0.15))),
        'z_gradient': float(dims[2] / max(0.01, np.sqrt(dims[0]**2 + dims[1]**2))),
        'aspect_ratio': float(dims[2] / max(0.05, min(dims[0], dims[1]))),
    }


def _classify_voxel(of: dict) -> tuple:
    """Rule-based classification from voxel-aggregated features.

    Returns (type_id, label).
    Tune thresholds via cluster_analyzer-like params.
    """
    R = of['mean_z_variance']       # roughness: higher = more scattered
    G = of['z_gradient']            # slope estimate
    H = float(of['dims'][2])        # height
    W = float(max(of['dims'][0], of['dims'][1]))
    W_min = float(min(of['dims'][0], of['dims'][1]))
    N = of['total_n']
    occ = of['occupancy']

    # sparse noise (low occupancy, scattered)
    if occ < 0.05 and N < 20:
        return TYPE_OBSTACLE, 'noise'

    # slope: smooth (low R), moderate gradient, substantial size
    if R < 0.04 and 0.01 < G < 0.5 and H < 3.0 and N > 20:
        direction = 'uphill' if of['dims'][2] > 0 else 'slope'
        return TYPE_SLOPE, f'slope_G{G:.2f}_H{H:.1f}m'

    # bump: very low height, smooth
    if H < 0.3 and R < 0.06:
        return TYPE_BUMP, f'bump_H{H:.2f}m'

    # pole: tall, thin, moderate roughness (vertical surfaces)
    aspect = H / max(0.05, W_min)
    if aspect > 2.5 and H > 0.3:
        return TYPE_POLE, f'pole_H{H:.1f}m'

    # obstacle: anything else
    return TYPE_OBSTACLE, f'obs_H{H:.1f}m'


# ---------------------------------------------------------------------------
# confidence scoring (reused from cluster_analyzer pattern)
# ---------------------------------------------------------------------------

def _confidence_score_voxel(of: dict, track_type_hist) -> float:
    """0-1 confidence from voxel features + temporal stability."""
    score = 0.0
    N = of['total_n']
    occ = of['occupancy']

    if N >= 50:   score += 0.3
    elif N >= 30: score += 0.25
    elif N >= 20: score += 0.2
    elif N >= 10: score += 0.1

    if occ > 0.3:  score += 0.2
    elif occ > 0.15: score += 0.1

    R = of['mean_z_variance']
    if R < 0.02 or R > 0.15:
        score += 0.2
    else:
        score += 0.1

    if track_type_hist and len(track_type_hist) >= 3:
        cnt = Counter(track_type_hist)
        stability = cnt.most_common(1)[0][1] / len(track_type_hist)
        score += 0.3 * stability

    return min(1.0, score)


# ---------------------------------------------------------------------------
# main node
# ---------------------------------------------------------------------------

class VoxelAnalyzer(Node):
    """Voxel-based geometric obstacle analyser."""

    def __init__(self):
        super().__init__('voxel_analyzer')

        self.declare_parameter('outlier_pct', 0.05)               # top % to remove per voxel
        self.declare_parameter('min_total_points', 10)            # min pts per object
        self.declare_parameter('min_dim', 0.10)
        self.declare_parameter('max_dim', 15.0)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('tracking_distance_threshold', 2.0)
        self.declare_parameter('tracking_history_size', 10)
        self.declare_parameter('tracking_max_lost', 3)
        self.declare_parameter('log_interval', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            PointCloud2, '/patchworkpp/nonground', self._callback, qos,
        )
        self.pub_boxes = self.create_publisher(
            MarkerArray, '/obstacles/boxes_3d_voxel', 10,
        )
        self.pub_centers = self.create_publisher(
            MarkerArray, '/obstacles/centers_3d_voxel', 10,
        )
        self.pub_low_conf = self.create_publisher(
            MarkerArray, '/lidar/low_confidence_voxel', 10,
        )

        self._frame_count = 0
        self._tracks = {}
        self._next_track_id = 1

        self.get_logger().info('Voxel Analyzer ready — multi-res grid + geometric features')

    def _match_tracks(self, centroids, type_ids, labels):
        """Greedy nearest-neighbour track matching (same as cluster_analyzer)."""
        dist_thr = self.get_parameter('tracking_distance_threshold').value
        hist_size = self.get_parameter('tracking_history_size').value
        max_lost = self.get_parameter('tracking_max_lost').value

        for t in self._tracks.values():
            t['_matched'] = False

        assignments = {}
        unassigned = list(range(len(centroids)))

        if self._tracks:
            track_ids = list(self._tracks.keys())
            track_cents = np.array([self._tracks[tid]['centroid'] for tid in track_ids])
            for ci, c in enumerate(centroids):
                dists = np.linalg.norm(track_cents - c, axis=1)
                best_j = int(np.argmin(dists))
                if dists[best_j] < dist_thr and not self._tracks[track_ids[best_j]]['_matched']:
                    assignments[ci] = track_ids[best_j]
                    self._tracks[track_ids[best_j]]['_matched'] = True
                    unassigned.remove(ci)

        for ci, tid in assignments.items():
            t = self._tracks[tid]
            t['centroid'] = centroids[ci]
            t['type_hist'].append(type_ids[ci])
            if len(t['type_hist']) > hist_size:
                t['type_hist'].popleft()
            t['label'] = labels[ci]
            t['type_id'] = Counter(t['type_hist']).most_common(1)[0][0]
            t['lost_count'] = 0

        for ci in unassigned:
            tid = self._next_track_id
            self._next_track_id += 1
            hist = deque([type_ids[ci]], maxlen=hist_size)
            self._tracks[tid] = {
                'centroid': centroids[ci], 'type_hist': hist,
                'type_id': type_ids[ci], 'label': labels[ci],
                'lost_count': 0, '_matched': True,
            }
            assignments[ci] = tid

        stale = [tid for tid, t in self._tracks.items() if not t['_matched']]
        for tid in stale:
            self._tracks[tid]['lost_count'] += 1
            if self._tracks[tid]['lost_count'] > max_lost:
                del self._tracks[tid]

        result = {}
        for ci in range(len(centroids)):
            tid = assignments.get(ci)
            if tid is not None and tid in self._tracks:
                result[ci] = (self._tracks[tid]['type_id'], self._tracks[tid]['label'])
            else:
                result[ci] = (type_ids[ci], labels[ci])
        return result

    def _callback(self, msg: PointCloud2):
        self._frame_count += 1
        xyz = _pc2_to_xyz(msg)
        if len(xyz) < 20:
            return

        pct = self.get_parameter('outlier_pct').value
        min_pts = self.get_parameter('min_total_points').value
        min_dim = self.get_parameter('min_dim').value
        max_dim = self.get_parameter('max_dim').value
        conf_thr = self.get_parameter('confidence_threshold').value
        log_int = self.get_parameter('log_interval').value

        # Step 1-2: multi-res grid + outlier removal
        grid = _build_multires_grid(xyz)
        grid = _remove_outliers(grid, xyz, pct)

        # Step 3: per-voxel features
        features = _voxel_features(grid, xyz)
        if not features:
            return

        # Step 4: voxel clustering → objects
        objects = _cluster_voxels(features)

        # Step 5: object features + classify
        now = self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id
        obj_info = []

        for obj in objects:
            if obj['total_n'] < min_pts:
                continue
            of = _object_features(obj, features, xyz)
            dmax = of['dims'].max()
            if dmax < min_dim or dmax > max_dim:
                continue
            type_id, label = _classify_voxel(of)
            obj_info.append((of, type_id, label))

        if not obj_info:
            return

        # Step 6: track + confidence + publish
        centroids_arr = np.array([oi[0]['centroid'] for oi in obj_info])
        raw_types = [oi[1] for oi in obj_info]
        raw_labels = [oi[2] for oi in obj_info]
        tracked = self._match_tracks(centroids_arr, raw_types, raw_labels)

        boxes_high = MarkerArray()
        centers_high = MarkerArray()
        boxes_low = MarkerArray()
        log_lines = []

        for i, (of, raw_type, raw_label) in enumerate(obj_info):
            stab_type, stab_label = tracked[i]

            track_hist = []
            for tid, t in self._tracks.items():
                if t.get('_matched') and np.linalg.norm(np.array(t['centroid']) - of['centroid']) < 0.1:
                    track_hist = list(t['type_hist'])
                    break

            conf = _confidence_score_voxel(of, track_hist)
            high_conf = conf >= conf_thr

            if self._frame_count % log_int == 0:
                log_lines.append(
                    f'[{TYPE_LABELS[stab_type]}] N={of["total_n"]} conf={conf:.2f} '
                    f'R={of["mean_z_variance"]:.3f} G={of["z_gradient"]:.3f} '
                    f'H={of["dims"][2]:.2f}m → {stab_label}'
                )

            color = TYPE_COLORS.get(stab_type, TYPE_COLORS[0])
            lbl = f'{stab_label} c{conf:.1f}'
            lifetime_ns = 300_000_000

            dims = of['dims']
            c = of['centroid']

            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = now
            box.ns = TYPE_LABELS[stab_type]
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(c[0])
            box.pose.position.y = float(c[1])
            box.pose.position.z = float(c[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(dims[0])
            box.scale.y = float(dims[1])
            box.scale.z = float(dims[2])
            box.color = color
            if not high_conf:
                box.color.a = 0.25
            box.lifetime.nanosec = lifetime_ns
            (boxes_high if high_conf else boxes_low).markers.append(box)

            center = Marker()
            center.header.frame_id = frame_id
            center.header.stamp = now
            center.ns = TYPE_LABELS[stab_type]
            center.id = i
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = float(c[0])
            center.pose.position.y = float(c[1])
            center.pose.position.z = float(c[2])
            center.scale.x = 0.25
            center.scale.y = 0.25
            center.scale.z = 0.25
            center.color = color
            if not high_conf:
                center.color.a = 0.25
            center.lifetime.nanosec = lifetime_ns
            center.text = lbl
            centers_high.markers.append(center)

        self.pub_boxes.publish(boxes_high)
        self.pub_centers.publish(centers_high)
        if boxes_low.markers:
            self.pub_low_conf.publish(boxes_low)

        if log_lines and self._frame_count % log_int == 0:
            self.get_logger().info(f'Frame {self._frame_count}: ' + ' | '.join(log_lines))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
