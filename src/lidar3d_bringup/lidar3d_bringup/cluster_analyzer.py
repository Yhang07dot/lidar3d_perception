#!/usr/bin/env python3
"""
Cluster geometry analyser — PCA feature extraction + rule-based classification.

Subscribes to /clusters/points_3d (PointCloud2 with intensity=cluster_id).
For each cluster computes PCA eigenvalues, planarity, slope angle, dimensions,
then classifies as: slope / bump / pole / obstacle.

Publishes:
  /obstacles/boxes_3d    — CUBE MarkerArray (axis-aligned, classified)
  /obstacles/centers_3d  — SPHERE MarkerArray (centroids with type label)

Colours:
  slope    → deep green  (0.0, 0.5, 0.0)
  bump     → yellow      (1.0, 1.0, 0.0)
  pole     → red         (1.0, 0.0, 0.0)
  obstacle → orange      (1.0, 0.5, 0.0)

Added 2026-07-29: slope-obstacle discrimination for off-road perception.
"""

from collections import Counter, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}

# --- classification constants ---
TYPE_OBSTACLE = 0
TYPE_POLE = 1
TYPE_BUMP = 2
TYPE_SLOPE = 3

TYPE_LABELS = {0: 'obstacle', 1: 'pole', 2: 'bump', 3: 'slope'}
TYPE_COLORS = {
    0: ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6),   # orange — obstacle
    1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),   # red — pole
    2: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5),   # yellow — bump
    3: ColorRGBA(r=0.0, g=0.5, b=0.0, a=0.45),  # deep green — slope
}


def _extract_clusters(xyz: np.ndarray, intensities: np.ndarray):
    """Group points by cluster_id (intensity). Returns list of (xyz_array, cid)."""
    clusters = []
    for cid in np.unique(intensities):
        cid_int = int(cid)
        if cid_int <= 0:
            continue
        mask = intensities == cid
        clusters.append((xyz[mask], cid_int))
    return clusters


def _pca_features(points: np.ndarray):
    """Compute PCA-based geometric features for a cluster.

    Returns dict with:
      eigenvalues  — sorted descending (λ0 ≥ λ1 ≥ λ2)
      normal       — principal axis of smallest variance
      planarity    — 1 - λ2/λ1  (1.0 = perfectly planar)
      linearity    — (λ1 - λ2) / λ0  (1.0 = perfectly linear)
      slope_angle_deg — angle between normal and vertical (0=flat, 90=wall)
      slope_azimuth_deg — horizontal direction of steepest descent
    """
    n = len(points)
    if n < 3:
        return None

    centered = points - points.mean(axis=0)
    # SVD of centered points: U @ diag(S) @ Vt
    # singular values S relate to eigenvalues: λ_i = S_i² / (n-1)
    # For ratios, S_i ratios equal sqrt(λ_i) ratios — good enough
    try:
        _, S, Vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None

    # squared singular values → proportional to eigenvalues
    lam = S ** 2
    # normalise
    lam_sum = lam.sum()
    if lam_sum < 1e-12:
        return None
    lam_norm = lam / lam_sum

    lam0, lam1, lam2 = lam_norm[0], lam_norm[1], lam_norm[2] if len(lam_norm) >= 3 else (lam_norm[0], lam_norm[1], 0.0)

    # normal vector = last row of Vt (smallest singular value direction)
    normal = Vt[-1] if len(Vt) >= 3 else Vt[-1]
    # ensure normal points upward (n_z > 0)
    if normal[2] < 0:
        normal = -normal

    # planarity: 1 - λ2/λ1.  Edge case λ1 ≈ 0 → clamp
    planarity = 1.0 - (lam2 / lam1) if lam1 > 1e-12 else 0.0
    planarity = max(0.0, min(1.0, planarity))

    # linearity: (λ1 - λ2) / λ0
    linearity = (lam1 - lam2) / lam0 if lam0 > 1e-12 else 0.0
    linearity = max(0.0, min(1.0, linearity))

    # slope angle: angle between normal and vertical (Z axis)
    nz_clamped = max(-1.0, min(1.0, float(normal[2])))
    slope_angle_rad = np.arccos(abs(nz_clamped))
    slope_angle_deg = float(np.degrees(slope_angle_rad))

    # slope azimuth: direction of steepest descent in XY plane
    # The gradient (steepest ascent) direction is the projection of the normal onto XY
    # Steepest descent is opposite
    nx, ny = float(normal[0]), float(normal[1])
    if abs(nx) > 1e-9 or abs(ny) > 1e-9:
        azimuth_rad = np.arctan2(-ny, -nx)  # descent direction
        slope_azimuth_deg = float(np.degrees(azimuth_rad))
    else:
        slope_azimuth_deg = 0.0

    return {
        'lam0': float(lam0), 'lam1': float(lam1), 'lam2': float(lam2),
        'normal': normal,
        'planarity': planarity,
        'linearity': linearity,
        'slope_angle_deg': slope_angle_deg,
        'slope_azimuth_deg': slope_azimuth_deg,
    }


def _classify(features: dict, dims: np.ndarray, n_points: int):
    """Rule-based classification from PCA features + AABB dimensions.

    dims = [width_x, width_y, height] (AABB spans)
    """
    if features is None:
        return TYPE_OBSTACLE, 'generic'

    P = features['planarity']
    L = features['linearity']
    alpha = features['slope_angle_deg']
    H = float(dims[2])       # height (Z span)
    W = float(max(dims[0], dims[1]))  # larger horizontal dimension
    W_min = float(min(dims[0], dims[1]))  # smaller horizontal dimension

    # --- slope: planar + moderate tilt + reasonable height ---
    if P > 0.85 and alpha < 15.0 and H < 2.0 and n_points > 20:
        direction = 'uphill' if features['normal'][2] > 0.3 else 'downhill'
        label = f'slope_{direction}_{alpha:.0f}deg'
        return TYPE_SLOPE, label

    # --- bump: low + somewhat planar ---
    if P > 0.7 and alpha < 15.0 and H < 0.3:
        return TYPE_BUMP, f'bump_{H:.2f}m'

    # --- pole: linear + tall + high aspect ratio ---
    aspect = H / W_min if W_min > 0.05 else H / 0.05
    if L > 0.7 and H > 0.3 and aspect > 2.5:
        return TYPE_POLE, f'pole_H{H:.1f}m'

    # --- low scattered bump ---
    if H < 0.25 and P < 0.7:
        return TYPE_BUMP, f'bump_{H:.2f}m'

    # --- anything else = obstacle ---
    return TYPE_OBSTACLE, f'obs_H{H:.1f}m'


class ClusterAnalyzer(Node):
    """PCA geometry analysis + classification with temporal tracking."""

    def __init__(self):
        super().__init__('cluster_analyzer')

        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('min_dim', 0.10)
        self.declare_parameter('max_dim', 15.0)
        self.declare_parameter('tracking_distance_threshold', 2.0)  # m, max centroid shift to match
        self.declare_parameter('tracking_history_size', 10)         # frames of history for mode voting
        self.declare_parameter('tracking_max_lost', 3)              # frames before removing stale track
        self.declare_parameter('log_interval', 10)                  # frames between summary logs

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            PointCloud2, '/clusters/points_3d', self._callback, qos,
        )
        self.pub_boxes = self.create_publisher(
            MarkerArray, '/obstacles/boxes_3d', 10,
        )
        self.pub_centers = self.create_publisher(
            MarkerArray, '/obstacles/centers_3d', 10,
        )

        self._frame_count = 0
        # Temporal tracking: track_id -> {centroid, type_hist (deque), label, lost_count}
        self._tracks = {}  # type: dict
        self._next_track_id = 1

        self.get_logger().info(
            'Cluster Analyzer ready — PCA + temporal tracking '
            f'(history={self.get_parameter("tracking_history_size").value}frames, '
            f'dist_thr={self.get_parameter("tracking_distance_threshold").value}m)'
        )

    def _match_tracks(self, centroids, type_ids, labels):
        """Greedy nearest-neighbour matching of new clusters to existing tracks."""
        from collections import deque

        dist_thr = self.get_parameter('tracking_distance_threshold').value
        hist_size = self.get_parameter('tracking_history_size').value
        max_lost = self.get_parameter('tracking_max_lost').value

        # --- mark all tracks as not-yet-matched this frame ---
        for t in self._tracks.values():
            t['_matched'] = False

        assignments = {}  # cluster_index -> track_id
        unassigned = list(range(len(centroids)))

        if not self._tracks:
            # no existing tracks — all clusters are new
            pass
        else:
            # build distance matrix
            track_ids = list(self._tracks.keys())
            track_centroids = np.array([self._tracks[tid]['centroid'] for tid in track_ids])

            for ci, c in enumerate(centroids):
                dists = np.linalg.norm(track_centroids - c, axis=1)
                best_j = int(np.argmin(dists))
                if dists[best_j] < dist_thr and not self._tracks[track_ids[best_j]]['_matched']:
                    assignments[ci] = track_ids[best_j]
                    self._tracks[track_ids[best_j]]['_matched'] = True
                    unassigned.remove(ci)

        # --- update matched tracks ---
        for ci, tid in assignments.items():
            t = self._tracks[tid]
            t['centroid'] = centroids[ci]
            t['type_hist'].append(type_ids[ci])
            if len(t['type_hist']) > hist_size:
                t['type_hist'].popleft()
            # majority vote
            from collections import Counter
            t['label'] = labels[ci]
            t['type_id'] = Counter(t['type_hist']).most_common(1)[0][0]
            t['lost_count'] = 0

        # --- create new tracks for unassigned clusters ---
        for ci in unassigned:
            tid = self._next_track_id
            self._next_track_id += 1
            hist = deque([type_ids[ci]], maxlen=hist_size)
            self._tracks[tid] = {
                'centroid': centroids[ci],
                'type_hist': hist,
                'type_id': type_ids[ci],
                'label': labels[ci],
                'lost_count': 0,
                '_matched': True,
            }
            assignments[ci] = tid

        # --- remove stale tracks ---
        stale = [tid for tid, t in self._tracks.items() if not t['_matched']]
        for tid in stale:
            self._tracks[tid]['lost_count'] += 1
            if self._tracks[tid]['lost_count'] > max_lost:
                del self._tracks[tid]

        # return dict of cluster_index -> (type_id, label)
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

        names = [f.name for f in msg.fields]
        formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
        offsets = [f.offset for f in msg.fields]
        dtype = np.dtype({
            'names': names, 'formats': formats,
            'offsets': offsets, 'itemsize': msg.point_step,
        })
        data = np.frombuffer(msg.data, dtype=dtype)
        xyz = np.column_stack([data['x'], data['y'], data['z']])
        intensities = data['intensity'] if 'intensity' in names else np.zeros(len(xyz))

        clusters = _extract_clusters(xyz, intensities)
        if not clusters:
            # mark all tracks as lost when no clusters
            for t in self._tracks.values():
                t['_matched'] = False
            return

        min_pts = self.get_parameter('min_cluster_points').value
        min_dim = self.get_parameter('min_dim').value
        max_dim = self.get_parameter('max_dim').value
        log_int = self.get_parameter('log_interval').value
        now = self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id

        # --- first pass: classify all clusters ---
        cluster_info = []  # (pts, cid, centroid, dims, type_id, label, features)
        for pts, cid in clusters:
            n_pts = len(pts)
            if n_pts < min_pts:
                continue
            min_pt = pts.min(axis=0)
            max_pt = pts.max(axis=0)
            centroid = (min_pt + max_pt) / 2.0
            dims = max_pt - min_pt
            dmax = dims.max()
            if dmax < min_dim or dmax > max_dim:
                continue
            features = _pca_features(pts)
            type_id, label = _classify(features, dims, n_pts)
            cluster_info.append((pts, cid, centroid, dims, type_id, label, features))

        if not cluster_info:
            return

        # --- second pass: temporal tracking ---
        centroids_arr = np.array([c[2] for c in cluster_info])
        raw_types = [c[4] for c in cluster_info]
        raw_labels = [c[5] for c in cluster_info]
        tracked = self._match_tracks(centroids_arr, raw_types, raw_labels)

        # --- third pass: publish markers with stabilized labels ---
        boxes = MarkerArray()
        centers = MarkerArray()
        log_lines = []

        for i, (pts, cid, centroid, dims, raw_type, raw_label, features) in enumerate(cluster_info):
            stab_type, stab_label = tracked[i]

            if self._frame_count % log_int == 0 and features:
                log_lines.append(
                    f'[{TYPE_LABELS[stab_type]}] cid={cid} '
                    f'P={features["planarity"]:.2f} L={features["linearity"]:.2f} '
                    f'slope={features["slope_angle_deg"]:.1f}deg '
                    f'H={dims[2]:.2f}m W={max(dims[0],dims[1]):.2f}m N={len(pts)} '
                    f'→ {stab_label}'
                )

            color = TYPE_COLORS.get(stab_type, TYPE_COLORS[0])
            lifetime_ns = 300_000_000

            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = now
            box.ns = TYPE_LABELS[stab_type]
            box.id = cid
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(centroid[0])
            box.pose.position.y = float(centroid[1])
            box.pose.position.z = float(centroid[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(dims[0])
            box.scale.y = float(dims[1])
            box.scale.z = float(dims[2])
            box.color = color
            box.lifetime.nanosec = lifetime_ns
            boxes.markers.append(box)

            center = Marker()
            center.header.frame_id = frame_id
            center.header.stamp = now
            center.ns = TYPE_LABELS[stab_type]
            center.id = cid
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = float(centroid[0])
            center.pose.position.y = float(centroid[1])
            center.pose.position.z = float(centroid[2])
            center.scale.x = 0.25
            center.scale.y = 0.25
            center.scale.z = 0.25
            center.color = color
            center.lifetime.nanosec = lifetime_ns
            center.text = stab_label
            centers.markers.append(center)

        self.pub_boxes.publish(boxes)
        self.pub_centers.publish(centers)

        if log_lines and self._frame_count % log_int == 0:
            self.get_logger().info(f'Frame {self._frame_count}: ' + ' | '.join(log_lines))


def main(args=None):
    rclpy.init(args=args)
    node = ClusterAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
