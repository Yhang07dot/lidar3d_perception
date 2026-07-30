#!/usr/bin/env python3
"""
Voxel-based obstacle analyser — boundary-first, multi-feature, pothole detection.

Subscribes:
  /patchworkpp/nonground   (PointCloud2, for obstacle analysis)
  /patchworkpp/ground      (PointCloud2, for pothole detection)

Publishes:
  /obstacles/boxes_3d_voxel    — high-confidence classified CUBE markers
  /lidar/low_confidence_voxel   — low-confidence debug markers
  /lidar/pothole_markers        — negative obstacle debug markers (purple SPHERES)

Architecture:
  1. Voxel-level features: z_range, roughness, step_height, ring_gradient, edge flag
  2. Boundary detection: z_range > edge_z_range → edge voxel
  3. 26-neighbour voxel flood-fill → objects
  4. Object-level PCA: slope, verticality, linearity, curvature, relative_elevation
  5. Multi-feature rule classification (7 rules + size sanity)
  6. Pothole: local Z anomaly in ground point cloud (2D grid, 4-neighbour)
  7. Temporal tracking + confidence scoring

ring recovery: computed from vertical angle (atan2(z, hypot(x,y))).
  ⚠ Gazebo bridge loses the native ring field. Real VLP-16 drivers include it natively.
  On real hardware, replace _compute_ring() with points['ring'] for zero-cost access.

Tuning: ros2 param set /voxel_analyzer <param> <value>
  Key thresholds: edge_z_range (default 0.15m), pothole_depth_m (0.08m)

Added 2026-07-30.
"""

import math
from collections import Counter, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}

# --- type constants ---
TYPE_OBSTACLE, TYPE_POLE, TYPE_BUMP, TYPE_SLOPE, TYPE_ROUGH, TYPE_POTHOLE = 0, 1, 2, 3, 4, 5
TYPE_LABELS = {0: 'obstacle', 1: 'pole', 2: 'bump', 3: 'slope', 4: 'rough', 5: 'pothole'}
# (r,g,b,a) tuples — use _mk_color() to create fresh ColorRGBA per marker
TYPE_COLORS = {
    0: (1.0, 0.5, 0.0, 0.6),   # orange — obstacle
    1: (1.0, 0.0, 0.0, 0.7),   # red — pole
    2: (1.0, 1.0, 0.0, 0.5),   # yellow — bump
    3: (0.0, 0.5, 0.0, 0.45),  # deep green — slope
    4: (1.0, 0.8, 0.0, 0.5),   # amber — rough terrain
    5: (0.5, 0.0, 1.0, 0.6),   # purple — pothole
}


def _mk_color(type_id: int, alpha_override: float = None) -> ColorRGBA:
    """Fresh ColorRGBA per marker (avoids shared-mutation across markers)."""
    r, g, b, a = TYPE_COLORS.get(type_id, TYPE_COLORS[0])
    if alpha_override is not None:
        a = alpha_override
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({'names': names, 'formats': formats,
                       'offsets': offsets, 'itemsize': msg.point_step})
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


# ⚠ Gazebo bridge drops native ring field. Compute from vertical angle as fallback.
# Real VLP-16 driver includes ring natively — use points['ring'] when available.
def _compute_ring(xyz: np.ndarray) -> np.ndarray:
    v_deg = np.degrees(np.arctan2(xyz[:, 2], np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2)))
    return np.clip(((v_deg + 15.0) / 2.0).astype(np.int32), 0, 15)


# ======================================================================
# Multi-resolution voxel grid
# ======================================================================

def _build_multires_grid(xyz: np.ndarray) -> dict:
    dist = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2)
    grid = {}
    for z_min, z_max, vs in [(0.0, 15.0, 0.10), (15.0, 30.0, 0.20), (30.0, 50.0, 0.40)]:
        mask = (dist >= z_min) & (dist < z_max)
        if not mask.any():
            continue
        pts = xyz[mask]
        indices = np.where(mask)[0]
        vx = np.floor(pts / vs).astype(np.int32)
        for vi in range(len(pts)):
            key = (vx[vi, 0], vx[vi, 1], vx[vi, 2])
            if key not in grid:
                grid[key] = {'pts': [], 'voxel_size': vs}
            grid[key]['pts'].append(indices[vi])
    return grid


def _remove_outliers(grid: dict, xyz: np.ndarray, pct: float = 0.05) -> dict:
    for cell in grid.values():
        idx = cell['pts']
        if len(idx) < 5:
            continue
        z = xyz[idx, 2]
        cutoff = np.percentile(z, 100 * (1.0 - pct))
        cell['pts'] = [i for i, zv in zip(idx, z) if zv <= cutoff]
    return grid


# ======================================================================
# Voxel-level features
# ======================================================================

def _voxel_features(grid: dict, xyz: np.ndarray, min_pts: int = 3) -> dict:
    r"""Compute z_range, roughness, density per voxel.  edge flag = z_range > 0.15m."""
    result = {}
    for key, cell in grid.items():
        idx = cell['pts']
        if len(idx) < min_pts:
            continue
        pts = xyz[idx]
        c = pts.mean(axis=0)
        zr = float(pts[:, 2].max() - pts[:, 2].min())
        cell['centroid'] = c
        cell['z_range'] = zr
        cell['roughness'] = float(np.mean((pts[:, 2] - c[2])**2))
        cell['n_pts'] = len(idx)
        cell['density'] = len(idx) / (cell['voxel_size']**3)
        cell['edge'] = zr > 0.15
        result[key] = cell
    return result


def _step_heights(features: dict) -> dict:
    """Max |Z diff| with 6 face-neighbours.  step > 0.1m → marks edge."""
    occ = set(features)
    offs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    for key, c in features.items():
        mx = 0.0
        for d in offs:
            n = (key[0] + d[0], key[1] + d[1], key[2] + d[2])
            if n in occ:
                mx = max(mx, abs(c['centroid'][2] - features[n]['centroid'][2]))
        c['step_height'] = mx
        if mx > 0.1:
            c['edge'] = True
    return features


def _ring_gradient(features: dict, xyz: np.ndarray) -> dict:
    """Max |Z diff| between adjacent rings within same voxel. O(n) via ring grouping."""
    rings = _compute_ring(xyz)
    for cell in features.values():
        idx = cell['pts']
        if len(idx) < 2:
            cell['ring_gradient'] = 0.0
            continue
        # group mean Z by ring
        ring_z = {}
        for i in idx:
            r = int(rings[i])
            ring_z.setdefault(r, []).append(xyz[i, 2])
        ring_means = {r: float(np.mean(zs)) for r, zs in ring_z.items()}
        mg = 0.0
        for r in ring_means:
            if r + 1 in ring_means:
                mg = max(mg, abs(ring_means[r] - ring_means[r + 1]))
        cell['ring_gradient'] = mg
    return features


# ======================================================================
# Voxel clustering
# ======================================================================

def _cluster_voxels(features: dict) -> list:
    occ = set(features)
    visited = set()
    objects = []
    offs = np.array([[dx, dy, dz] for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                     if not (dx == 0 and dy == 0 and dz == 0)], dtype=np.int32)

    for seed in occ:
        if seed in visited:
            continue
        visited.add(seed)
        q, vx = [seed], [seed]
        while q:
            cur = q.pop()
            for o in offs:
                nb = (cur[0] + o[0], cur[1] + o[1], cur[2] + o[2])
                if nb in occ and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
                    vx.append(nb)
        pts = []
        cents = []
        for v in vx:
            pts.extend(features[v]['pts'])
            cents.append(features[v]['centroid'])
        objects.append({'voxel_keys': vx, 'all_point_indices': pts,
                        'total_n': len(pts), 'n_voxels': len(vx),
                        'centroids': np.array(cents), 'mean_centroid': np.array(cents).mean(axis=0)})
    return objects


# ======================================================================
# Object-level PCA
# ======================================================================

def _object_pca(pts: np.ndarray) -> dict:
    n = len(pts)
    if n < 5:
        return {'slope_deg': 0, 'verticality': 0, 'linearity': 0, 'curvature': 0}
    c = pts - pts.mean(axis=0)
    try:
        _, S, Vt = np.linalg.svd(c, full_matrices=False)
    except np.linalg.LinAlgError:
        return {'slope_deg': 0, 'verticality': 0, 'linearity': 0, 'curvature': 0}
    lam = S**2
    s = lam.sum()
    if s < 1e-12:
        return {'slope_deg': 0, 'verticality': 0, 'linearity': 0, 'curvature': 0}
    l0 = lam[0] / s
    l1 = lam[1] / s if len(lam) >= 2 else 0
    l2 = lam[2] / s if len(lam) >= 3 else 0
    nz = max(-1.0, min(1.0, float(Vt[-1][2])))
    if nz < 0:
        nz = -nz
    return {
        'slope_deg': float(math.degrees(math.acos(nz))),
        'verticality': 1.0 - nz,
        'linearity': (l0 - l1) / l0 if l0 > 1e-12 else 0.0,
        'curvature': l2,
    }


# ======================================================================
# Object features + classification
# ======================================================================

def _object_features(obj: dict, feat: dict, xyz: np.ndarray, ground_z: float = None) -> dict:
    zr = [feat[k]['z_range'] for k in obj['voxel_keys']]
    sh = [feat[k]['step_height'] for k in obj['voxel_keys']]
    ro = [feat[k]['roughness'] for k in obj['voxel_keys']]
    ed = [feat[k]['edge'] for k in obj['voxel_keys']]
    rg = [feat[k]['ring_gradient'] for k in obj['voxel_keys']]

    pts = xyz[obj['all_point_indices']]
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    dims = mx - mn
    pca = _object_pca(pts)

    rel = 0.0
    if ground_z is not None:
        rel = float(mn[2] - ground_z)

    return {
        'dims': dims, 'centroid': (mn + mx) / 2.0,
        'total_n': obj['total_n'], 'n_voxels': obj['n_voxels'],
        'mean_z_range': float(np.mean(zr)),
        'mean_step_height': float(np.mean(sh)),
        'mean_roughness': float(np.mean(ro)),
        'edge_ratio': float(np.mean(ed)),
        'max_ring_gradient': float(max(rg)) if rg else 0.0,
        **pca,
        'relative_elevation': rel,
        'aspect': float(dims[2] / max(0.05, min(dims[0], dims[1]))),
        'max_dim': float(dims.max()),
        'width': float(max(dims[0], dims[1])),
        'width_min': float(min(dims[0], dims[1])),
    }


def _classify(of: dict) -> tuple:
    """Boundary-first classification: obstacles first, then passable terrain."""
    H = float(of['dims'][2])

    # ---- 0. size sanity: too large = terrain ----
    if of['max_dim'] > 8.0 or (of['width'] > 6.0 and of['slope_deg'] < 20):
        return TYPE_SLOPE, f'slope_big_H{H:.1f}m'

    # ==== OBSTACLE DETECTION (boundary-first) ====

    # 1. sharp boundary + meaningful height → obstacle edge
    if of['edge_ratio'] > 0.25 and H > 0.2:
        return TYPE_OBSTACLE, f'obs_edge{of["edge_ratio"]:.1f}_H{H:.1f}m'

    # 2. elevated above local ground → floating obstacle
    if of['relative_elevation'] > 0.15 and of['total_n'] > 10:
        return TYPE_OBSTACLE, f'obs_elev{of["relative_elevation"]:.2f}m'

    # 3. sudden vertical step → obstacle boundary
    if of['mean_step_height'] > 0.12 and H > 0.3:
        return TYPE_OBSTACLE, f'obs_step{of["mean_step_height"]:.2f}_H{H:.1f}m'

    # 4. high verticality + tall → wall-like obstacle
    if of['verticality'] > 0.75 and H > 0.5:
        return TYPE_OBSTACLE, f'obs_vert{of["verticality"]:.2f}_H{H:.1f}m'

    # ==== PASSABLE TERRAIN (obstacles ruled out) ====

    # 5. pole: vertical, tall, thin (not obstacle because sparse & isolated)
    if of['verticality'] > 0.85 and H > 0.3 and of['width_min'] < 0.5:
        return TYPE_POLE, f'pole_H{H:.1f}m'

    # 6. bump: low, sharp ring gradient or curvature
    if H < 0.3 and (of['max_ring_gradient'] > 0.05 or of['curvature'] > 0.02):
        return TYPE_BUMP, f'bump_H{H:.2f}m'

    # 7. slope: gentle, smooth, low edge
    if of['slope_deg'] < 15.0 and of['mean_step_height'] < 0.08 and of['edge_ratio'] < 0.2:
        return TYPE_SLOPE, f'slope_{of["slope_deg"]:.0f}deg'

    # 8. rough: bumpy but not steep
    if of['mean_roughness'] > 0.01 and of['slope_deg'] < 10.0:
        return TYPE_ROUGH, f'rough_R{of["mean_roughness"]:.3f}'

    # fallback
    return TYPE_OBSTACLE, f'obs_H{H:.1f}m'


# ======================================================================
# Pothole detection (ground post-processing)
# ======================================================================

def _detect_potholes(ground_xyz: np.ndarray, grid_res: float = 0.2,
                     depth_thr: float = 0.08, min_pts: int = 3) -> list:
    if len(ground_xyz) < 20:
        return []
    x, y, z = ground_xyz[:, 0], ground_xyz[:, 1], ground_xyz[:, 2]
    ix = np.floor(x / grid_res).astype(np.int32)
    iy = np.floor(y / grid_res).astype(np.int32)
    cells = {}
    for i in range(len(ground_xyz)):
        k = (ix[i], iy[i])
        cells.setdefault(k, []).append(z[i])
    mean_z = {k: float(np.mean(zs)) for k, zs in cells.items() if len(zs) >= min_pts}
    pots = []
    for k, mz in mean_z.items():
        dep = max((mean_z.get((k[0] + d[0], k[1] + d[1]), mz) - mz
                   for d in [(1, 0), (-1, 0), (0, 1), (0, -1)]), default=0)
        if dep > depth_thr:
            pots.append((k[0] * grid_res + grid_res / 2, k[1] * grid_res + grid_res / 2, dep))
    return pots


# ======================================================================
# Confidence
# ======================================================================

def _confidence(of: dict, track_hist) -> float:
    s = 0.0
    N = of['total_n']
    if N >= 50:   s += 0.3
    elif N >= 30: s += 0.25
    elif N >= 20: s += 0.2
    elif N >= 10: s += 0.1
    s += 0.2 if (of['edge_ratio'] > 0.3 or of['edge_ratio'] < 0.05) else 0.1
    s += 0.2 if (of['slope_deg'] < 5 or of['slope_deg'] > 20) else 0.0
    if track_hist and len(track_hist) >= 3:
        s += 0.3 * Counter(track_hist).most_common(1)[0][1] / len(track_hist)
    return min(1.0, s)


# ======================================================================
# Main node
# ======================================================================

class VoxelAnalyzer(Node):
    def __init__(self):
        super().__init__('voxel_analyzer')

        for p, v in [('outlier_pct', 0.05), ('min_total_points', 10), ('min_dim', 0.10),
                      ('max_dim', 15.0), ('confidence_threshold', 0.35),
                      ('tracking_distance_threshold', 2.0), ('tracking_history_size', 10),
                      ('tracking_max_lost', 3), ('log_interval', 10), ('edge_z_range', 0.15),
                      ('pothole_depth_m', 0.08)]:
            self.declare_parameter(p, v)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ng = self.create_subscription(PointCloud2, '/patchworkpp/nonground', self._cb_ng, qos)
        self.sub_g = self.create_subscription(PointCloud2, '/patchworkpp/ground', self._cb_g, qos)
        self.pub_boxes = self.create_publisher(MarkerArray, '/obstacles/boxes_3d_voxel', 10)
        self.pub_cents = self.create_publisher(MarkerArray, '/obstacles/centers_3d_voxel', 10)
        self.pub_low = self.create_publisher(MarkerArray, '/lidar/low_confidence_voxel', 10)
        self.pub_pot = self.create_publisher(MarkerArray, '/lidar/pothole_markers', 10)

        self._fc, self._tracks, self._tid, self._gz = 0, {}, 1, None
        self.get_logger().info('Voxel Analyzer v2 — boundary-first + 7 features + pothole')

    def _match_tracks(self, cents, tids, labels):
        dthr = self.get_parameter('tracking_distance_threshold').value
        hsz = self.get_parameter('tracking_history_size').value
        mlost = self.get_parameter('tracking_max_lost').value
        for t in self._tracks.values():
            t['_m'] = False
        a, ua = {}, list(range(len(cents)))
        if self._tracks:
            tks = list(self._tracks)
            tc = np.array([self._tracks[t]['centroid'] for t in tks])
            for ci, c in enumerate(cents):
                ds = np.linalg.norm(tc - c, axis=1)
                bj = int(np.argmin(ds))
                if ds[bj] < dthr and not self._tracks[tks[bj]]['_m']:
                    a[ci] = tks[bj]; self._tracks[tks[bj]]['_m'] = True; ua.remove(ci)
        for ci, tid in a.items():
            t = self._tracks[tid]; t['centroid'] = cents[ci]; t['th'].append(tids[ci])
            if len(t['th']) > hsz: t['th'].popleft()
            t['label'] = labels[ci]; t['type_id'] = Counter(t['th']).most_common(1)[0][0]; t['lc'] = 0
        for ci in ua:
            tid = self._tid; self._tid += 1
            self._tracks[tid] = {'centroid': cents[ci], 'th': deque([tids[ci]], maxlen=hsz),
                                  'type_id': tids[ci], 'label': labels[ci], 'lc': 0, '_m': True}
            a[ci] = tid
        for tid in [t for t in self._tracks if not self._tracks[t]['_m']]:
            self._tracks[tid]['lc'] += 1
            if self._tracks[tid]['lc'] > mlost: del self._tracks[tid]
        result = {}
        for ci in range(len(cents)):
            tid = a.get(ci)
            if tid is not None and tid in self._tracks:
                result[ci] = (self._tracks[tid]['type_id'], self._tracks[tid]['label'])
            else:
                result[ci] = (tids[ci], labels[ci])
        return result

    def _cb_g(self, msg: PointCloud2):
        g = _pc2_to_xyz(msg)
        if len(g) < 20: return
        self._gz = float(g[:, 2].mean())
        pots = _detect_potholes(g, depth_thr=self.get_parameter('pothole_depth_m').value)
        if pots:
            ma = MarkerArray(); now = self.get_clock().now().to_msg()
            for i, (cx, cy, d) in enumerate(pots):
                m = Marker(header=Header(frame_id=msg.header.frame_id, stamp=now),
                           ns='pothole', id=i, type=Marker.SPHERE, action=Marker.ADD)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -d / 2
                m.scale.x = m.scale.y = m.scale.z = 0.3; m.color = _mk_color(TYPE_POTHOLE)
                m.lifetime.nanosec = 500_000_000; m.text = f'pothole_{d*100:.0f}cm'
                ma.markers.append(m)
            self.pub_pot.publish(ma)

    def _cb_ng(self, msg: PointCloud2):
        self._fc += 1; xyz = _pc2_to_xyz(msg)
        if len(xyz) < 20: return

        grid = _remove_outliers(_build_multires_grid(xyz), xyz, self.get_parameter('outlier_pct').value)
        feat = _voxel_features(grid, xyz)
        if not feat: return
        feat = _step_heights(feat)
        feat = _ring_gradient(feat, xyz)

        objs = _cluster_voxels(feat)
        now = self.get_clock().now().to_msg(); fid = msg.header.frame_id
        oi = []

        for obj in objs:
            if obj['total_n'] < self.get_parameter('min_total_points').value: continue
            of = _object_features(obj, feat, xyz, self._gz)
            if of['dims'].max() < self.get_parameter('min_dim').value: continue
            if of['dims'].max() > self.get_parameter('max_dim').value: continue
            t, l = _classify(of); oi.append((of, t, l))

        if not oi: return

        tracked = self._match_tracks(np.array([x[0]['centroid'] for x in oi]),
                                      [x[1] for x in oi], [x[2] for x in oi])
        cthr = self.get_parameter('confidence_threshold').value; li = self.get_parameter('log_interval').value
        bh, ch, bl = MarkerArray(), MarkerArray(), MarkerArray(); ll = []

        for i, (of, _, _) in enumerate(oi):
            st, sl = tracked[i]
            th = []
            for t in self._tracks.values():
                if t.get('_m') and np.linalg.norm(np.array(t['centroid']) - of['centroid']) < 0.1:
                    th = list(t['th']); break
            c = _confidence(of, th); hi = c >= cthr

            if self._fc % li == 0:
                ll.append(f'[{TYPE_LABELS[st]}] N={of["total_n"]} c={c:.2f} '
                          f'E={of["edge_ratio"]:.2f} S={of["slope_deg"]:.1f}deg '
                          f'V={of["verticality"]:.2f} H={of["dims"][2]:.2f}m → {sl}')

            dm, ct = of['dims'], of['centroid']; lt = 300_000_000

            box = Marker(header=Header(frame_id=fid, stamp=now), ns=TYPE_LABELS[st], id=i,
                         type=Marker.CUBE, action=Marker.ADD)
            box.pose.position.x, box.pose.position.y, box.pose.position.z = float(ct[0]), float(ct[1]), float(ct[2])
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = float(dm[0]), float(dm[1]), float(dm[2])
            box.color = _mk_color(st, 0.25 if not hi else None); box.lifetime.nanosec = lt
            (bh if hi else bl).markers.append(box)

            ctr = Marker(header=Header(frame_id=fid, stamp=now), ns=TYPE_LABELS[st], id=i,
                         type=Marker.SPHERE, action=Marker.ADD)
            ctr.pose.position.x, ctr.pose.position.y, ctr.pose.position.z = float(ct[0]), float(ct[1]), float(ct[2])
            ctr.scale.x = ctr.scale.y = ctr.scale.z = 0.25
            ctr.color = _mk_color(st, 0.25 if not hi else None)
            ctr.lifetime.nanosec = lt; ctr.text = f'{sl} c{c:.1f}'; ch.markers.append(ctr)

        self.pub_boxes.publish(bh); self.pub_cents.publish(ch)
        if bl.markers: self.pub_low.publish(bl)
        if ll and self._fc % li == 0:
            self.get_logger().info(f'F{self._fc}: ' + ' | '.join(ll))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelAnalyzer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
