#!/usr/bin/env python3
"""
Two-layer terrain-aware obstacle detector.

Layer 1 — Surface model:
  Build a smoothed 2.5D polar grid from Patchwork++ ground points.
  The surface S(r,θ) provides the "expected terrain height" at every location.

Layer 2 — Residual analysis:
  For every non-ground point, compute residual = z - S(r,θ).
  Only points significantly above the surface are flagged as obstacles.
  Cluster flagged points and classify by geometry.

Key advantage over voxel_analyzer: the surface model provides CONTEXT.
A rough patch on a slope has near-zero residual (it follows the surface);
a true obstacle has large residual (it sticks above the surface).

Subscribes:
  /patchworkpp/ground      (PointCloud2)
  /patchworkpp/nonground    (PointCloud2)

Publishes:
  /obstacles/boxes_3d_surface    — CUBE MarkerArray (high confidence)
  /lidar/low_confidence_surface   — CUBE MarkerArray (low confidence, debug)
  /lidar/pothole_markers          — pothole detections

Added 2026-07-31: surface-fitting approach replacing bottom-up voxel classification.
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

# scipy optional — used for Gaussian smoothing of the surface grid
try:
    from scipy.ndimage import gaussian_filter
    HAS_SCIPY = True
except Exception:
    HAS_SCIPY = False  # NumPy 2.x / SciPy incompatibility → fallback box blur

DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}

# --- type system ---
TYPE_OBSTACLE, TYPE_POLE, TYPE_BUMP, TYPE_SLOPE = 0, 1, 2, 3
TYPE_ROUGH, TYPE_POTHOLE, TYPE_WAVE, TYPE_BOUNDARY = 4, 5, 6, 7
TYPE_LABELS = {0: 'obstacle', 1: 'pole', 2: 'bump', 3: 'slope',
               4: 'rough', 5: 'pothole', 6: 'wave', 7: 'boundary'}
TYPE_COLORS = {
    0: (1.0, 0.5, 0.0, 0.6),   # orange
    1: (1.0, 0.0, 0.0, 0.7),   # red
    2: (1.0, 1.0, 0.0, 0.5),   # yellow
    3: (0.0, 0.5, 0.0, 0.45),  # deep green
    4: (1.0, 0.8, 0.0, 0.5),   # amber
    5: (0.5, 0.0, 1.0, 0.6),   # purple
    6: (0.0, 0.7, 1.0, 0.5),   # cyan
    7: (1.0, 0.6, 0.8, 0.6),   # pink
}


def _mk_color(type_id: int, alpha_override: float = None) -> ColorRGBA:
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


# ======================================================================
# Layer 1 — Surface model
# ======================================================================

def _build_polar_grid(xyz: np.ndarray, r_min: float = 0.5, r_max: float = 35.0,
                      dr_base: float = 0.10, dr_per_m: float = 0.02,
                      dth_deg: float = 1.5) -> dict:
    """Bin ground points into a polar grid (r, theta) in vehicle frame.

    Grid spacing grows with range: dr = dr_base + dr_per_m * r
    Returns: {'z_median': 2D array, 'z_mad': 2D array, 'count': 2D array,
              'r_edges', 'th_edges', 'r_centers', 'th_centers', 'n_r', 'n_th'}
    """
    r = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2)
    th = np.arctan2(xyz[:, 1], xyz[:, 0])
    z = xyz[:, 2]

    mask = (r >= r_min) & (r <= r_max)
    r, th, z = r[mask], th[mask], z[mask]
    if len(r) < 10:
        return None

    # radial bins with growing spacing
    r_edges = [r_min]
    while r_edges[-1] < r_max:
        dr = dr_base + dr_per_m * r_edges[-1]
        r_edges.append(r_edges[-1] + dr)
    r_edges = np.array(r_edges)
    n_r = len(r_edges) - 1

    # angular bins
    th_edges = np.linspace(-math.pi, math.pi, int(360 / dth_deg) + 1)
    n_th = len(th_edges) - 1

    ir = np.clip(np.digitize(r, r_edges) - 1, 0, n_r - 1)
    ith = np.clip(np.digitize(th, th_edges) - 1, 0, n_th - 1)

    z_median = np.full((n_r, n_th), np.nan)
    z_mad = np.full((n_r, n_th), np.nan)
    count = np.zeros((n_r, n_th), dtype=np.int32)

    for ri in range(n_r):
        for ti in range(n_th):
            m = (ir == ri) & (ith == ti)
            if m.sum() >= 3:
                zs = z[m]
                med = np.median(zs)
                z_median[ri, ti] = med
                z_mad[ri, ti] = 1.4826 * np.median(np.abs(zs - med))
                count[ri, ti] = m.sum()

    r_centers = 0.5 * (r_edges[:-1] + r_edges[1:])
    th_centers = 0.5 * (th_edges[:-1] + th_edges[1:])

    return {'z_median': z_median, 'z_mad': z_mad, 'count': count,
            'r_edges': r_edges, 'th_edges': th_edges,
            'r_centers': r_centers, 'th_centers': th_centers,
            'n_r': n_r, 'n_th': n_th}


def _fill_and_smooth(grid: dict, sigma: float = 1.0) -> np.ndarray:
    """Fill NaN cells by nearest-neighbour propagation, then Gaussian smooth."""
    z = grid['z_median'].copy()
    n_r, n_th = z.shape

    # simple fill: for each NaN, take mean of non-NaN neighbours (iterative)
    for _ in range(3):
        for ri in range(n_r):
            for ti in range(n_th):
                if np.isnan(z[ri, ti]):
                    nbrs = []
                    for dr, dt in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                        nr, nt = ri + dr, (ti + dt) % n_th
                        if 0 <= nr < n_r and not np.isnan(z[nr, nt]):
                            nbrs.append(z[nr, nt])
                    if nbrs:
                        z[ri, ti] = np.mean(nbrs)

    # fill remaining with interpolation along r
    for ti in range(n_th):
        col = z[:, ti]
        valid = ~np.isnan(col)
        if valid.sum() >= 2:
            z[~valid, ti] = np.interp(
                np.where(~valid)[0], np.where(valid)[0], col[valid])

    # Gaussian smooth
    if HAS_SCIPY:
        z = gaussian_filter(z, sigma=sigma, mode='nearest')
    else:
        # fallback: simple box blur
        for _ in range(int(sigma)):
            z_pad = np.pad(z, 1, mode='edge')
            z = (z_pad[:-2, 1:-1] + z_pad[2:, 1:-1] +
                 z_pad[1:-1, :-2] + z_pad[1:-1, 2:]) / 4.0

    return z


def _sample_surface(S: np.ndarray, grid: dict, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Bilinear sample the surface grid at (x, y) positions. Returns expected z."""
    r = np.sqrt(x**2 + y**2)
    th = np.arctan2(y, x)
    r_edges = grid['r_edges']
    th_edges = grid['th_edges']
    n_r, n_th = S.shape

    ir = np.clip(np.searchsorted(r_edges, r) - 1, 0, n_r - 1)
    ith = np.clip(np.searchsorted(th_edges, th) - 1, 0, n_th - 1)

    return S[ir, ith]


def _adaptive_threshold(r: np.ndarray, th_near: float = 0.15,
                        th_far: float = 0.40, r_near: float = 5.0,
                        r_far: float = 30.0) -> np.ndarray:
    """Obstacle height threshold that grows with range."""
    t = np.clip((r - r_near) / (r_far - r_near), 0.0, 1.0)
    return th_near + t * (th_far - th_near)


# ======================================================================
# Layer 2 — Residual analysis + classification
# ======================================================================

def _cluster_residual_pts(xyz: np.ndarray, residuals: np.ndarray,
                          threshold: np.ndarray, min_pts: int = 5) -> list:
    """Cluster points with residual > threshold using 2D connected components."""
    mask = residuals > threshold
    if mask.sum() < min_pts:
        return []

    pts = xyz[mask]
    # 2D grid-based connected components
    grid_res = 0.2
    ix = np.floor(pts[:, 0] / grid_res).astype(np.int32)
    iy = np.floor(pts[:, 1] / grid_res).astype(np.int32)

    cells = {}
    for i in range(len(pts)):
        k = (ix[i], iy[i])
        cells.setdefault(k, []).append(i)

    occupied = set(cells)
    visited = set()
    clusters = []
    offs = [(1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, -1), (1, -1), (-1, 1)]

    for seed in occupied:
        if seed in visited:
            continue
        visited.add(seed)
        q, comp = [seed], [seed]
        while q:
            cur = q.pop()
            for o in offs:
                nb = (cur[0] + o[0], cur[1] + o[1])
                if nb in occupied and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
                    comp.append(nb)

        idxs = []
        for c in comp:
            idxs.extend(cells[c])
        if len(idxs) >= min_pts:
            clusters.append(pts[idxs])
    return clusters


def _classify_surface(pts: np.ndarray, S_z: np.ndarray, ground_z_mean: float) -> tuple:
    """Classify a residual-point cluster based on its geometric properties."""
    n = len(pts)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    dims = mx - mn
    c = (mn + mx) / 2.0
    H = float(dims[2])

    # PCA for shape
    if n >= 5:
        try:
            centered = pts - c
            _, S_vals, Vt = np.linalg.svd(centered, full_matrices=False)
            lam = S_vals**2
            ls = lam.sum()
            if ls > 1e-12:
                l0, l1, l2 = lam[0] / ls, lam[1] / ls, lam[2] / ls
                nz = abs(float(Vt[-1][2]))
                verticality = 1.0 - nz
                slope_deg = float(math.degrees(math.acos(max(-1.0, min(1.0, nz)))))
            else:
                verticality, slope_deg, l0, l1, l2 = 0.0, 0.0, 1.0, 0.0, 0.0
        except np.linalg.LinAlgError:
            verticality, slope_deg, l0, l1, l2 = 0.0, 0.0, 1.0, 0.0, 0.0
    else:
        verticality, slope_deg, l0, l1, l2 = 0.0, 0.0, 1.0, 0.0, 0.0

    W, W_min = float(max(dims[0], dims[1])), float(min(dims[0], dims[1]))
    aspect = H / max(0.05, W_min)
    linearity = (l0 - l1) / l0 if l0 > 1e-12 else 0.0
    curvature = l2
    rel_elev = float(mn[2] - ground_z_mean) if ground_z_mean is not None else 0.0

    # ---- classification ----
    if W > 6.0:
        return TYPE_SLOPE, 'slope_big', c, dims
    if verticality > 0.85 and H > 0.3 and W_min < 0.5:
        return TYPE_POLE, f'pole_H{H:.1f}m', c, dims
    if H < 0.3 and curvature > 0.02:
        return TYPE_BUMP, f'bump_H{H:.2f}m', c, dims
    if slope_deg < 15.0 and linearity < 0.5 and H < 2.0:
        return TYPE_SLOPE, f'slope_{slope_deg:.0f}deg', c, dims
    if rel_elev > 0.2 and H > 0.3:
        return TYPE_OBSTACLE, f'obs_elev{rel_elev:.2f}m', c, dims
    if n > 30 and curvature < 0.01:
        return TYPE_WAVE, f'wave_H{H:.2f}m', c, dims
    if verticality > 0.6 and H > 0.4:
        return TYPE_OBSTACLE, f'obs_H{H:.1f}m', c, dims
    return TYPE_ROUGH, f'rough_H{H:.2f}m', c, dims


def _confidence_surface(n_pts: int, verticality: float, edge_ratio: float,
                        track_hist) -> float:
    s = 0.0
    if n_pts >= 30:   s += 0.3
    elif n_pts >= 20: s += 0.2
    elif n_pts >= 10: s += 0.1
    s += 0.2 if (verticality > 0.8 or verticality < 0.1) else 0.1
    s += 0.2 if (edge_ratio > 0.5 or edge_ratio < 0.1) else 0.0
    if track_hist and len(track_hist) >= 3:
        s += 0.3 * Counter(track_hist).most_common(1)[0][1] / len(track_hist)
    return min(1.0, s)


# ======================================================================
# Pothole detection
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
# Main node
# ======================================================================

class SurfaceDetector(Node):
    def __init__(self):
        super().__init__('surface_detector')

        # ==== 曲面模型 (Layer 1) ====
        # 极坐标网格: dr = dr_base + dr_per_m * r  (近处细, 远处粗)
        self.declare_parameter('grid_dr_base', 0.10)     # 径向基础分辨率(m)
        self.declare_parameter('grid_dr_per_m', 0.02)    # 径向分辨率增长率(m/m)
        self.declare_parameter('grid_dth_deg', 1.5)       # 角分辨率(度)
        # 曲面平滑: ↑=曲面更平滑, 不跟随小突起, 矮障碍物残留差更大
        self.declare_parameter('smooth_sigma', 1.0)       # 高斯平滑σ(格数). 0.5=紧贴地形, 2.0=强抹平

        # ==== 残差分析 (Layer 2) ====
        # 障碍物阈值: residual = z_实际 - S(地面). 降低→更敏感(矮障碍物可检出, 但噪点增多)
        self.declare_parameter('residual_th_near', 0.15)  # 近处(5m)障碍物高度阈值(m)
        self.declare_parameter('residual_th_far', 0.40)   # 远处(30m)障碍物高度阈值(m)
        # 噪声抑制: residual > mad_factor × 局部MAD 才触发 (过滤草坪/碎石高方差)
        self.declare_parameter('mad_factor', 3.0)         # MAD倍率. ↓=更敏感, ↑=更抗噪
        # 聚类: 残差点2D连通域. ↓=小物体可检出但碎片增多
        self.declare_parameter('min_cluster_pts', 8)       # 最小聚类点数

        # ==== 置信度 & 发布 ====
        self.declare_parameter('confidence_threshold', 0.35)  # ≥此值→高置信发布. ↓=少过滤, ↑=多过滤
        self.declare_parameter('pothole_depth_m', 0.08)       # 坑洼深度阈值(m). ground局部Z异常

        # ==== 时序追踪 ====
        self.declare_parameter('track_dist_thr', 2.0)     # 跨帧匹配最大质心距离(m)
        self.declare_parameter('track_hist', 10)           # 历史帧数(众数投票窗口)
        self.declare_parameter('track_max_lost', 3)        # 丢帧上限(超过→删除track)
        self.declare_parameter('log_interval', 10)          # 日志输出帧间隔

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_g = self.create_subscription(PointCloud2, '/patchworkpp/ground', self._cb_ground, qos)
        self.sub_ng = self.create_subscription(PointCloud2, '/patchworkpp/nonground', self._cb_nonground, qos)

        self.pub_boxes = self.create_publisher(MarkerArray, '/obstacles/boxes_3d_surface', 10)
        self.pub_low = self.create_publisher(MarkerArray, '/lidar/low_confidence_surface', 10)
        self.pub_pot = self.create_publisher(MarkerArray, '/lidar/pothole_markers', 10)

        self._surface = None
        self._grid = None
        self._ground_z = None
        self._tracks = {}
        self._tid = 1
        self._fc = 0

        self.get_logger().info('Surface Detector ready — polar grid + surface model + residuals')

    def _match_tracks(self, cents, tids, labels):
        dthr = self.get_parameter('track_dist_thr').value
        hsz = self.get_parameter('track_hist').value
        ml = self.get_parameter('track_max_lost').value
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
            if self._tracks[tid]['lc'] > ml: del self._tracks[tid]
        r = {}
        for ci in range(len(cents)):
            tid = a.get(ci)
            if tid is not None and tid in self._tracks:
                r[ci] = (self._tracks[tid]['type_id'], self._tracks[tid]['label'])
            else:
                r[ci] = (tids[ci], labels[ci])
        return r

    def _cb_ground(self, msg: PointCloud2):
        g = _pc2_to_xyz(msg)
        n = len(g)
        if n < 20:
            return
        self._ground_z = float(g[:, 2].mean())

        # build polar surface grid
        grid = _build_polar_grid(g,
                                 dr_base=self.get_parameter('grid_dr_base').value,
                                 dr_per_m=self.get_parameter('grid_dr_per_m').value,
                                 dth_deg=self.get_parameter('grid_dth_deg').value)
        if grid is None:
            return
        S = _fill_and_smooth(grid, sigma=self.get_parameter('smooth_sigma').value)
        self._surface = S
        self._grid = grid

        # pothole detection
        pots = _detect_potholes(g, depth_thr=self.get_parameter('pothole_depth_m').value)
        if pots:
            ma = MarkerArray(); now = self.get_clock().now().to_msg()
            for i, (cx, cy, d) in enumerate(pots):
                m = Marker(header=Header(frame_id=msg.header.frame_id, stamp=now),
                           ns='pothole', id=i, type=Marker.SPHERE, action=Marker.ADD)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -d / 2
                m.scale.x = m.scale.y = m.scale.z = 0.3
                m.color = _mk_color(TYPE_POTHOLE); m.lifetime.nanosec = 500_000_000
                m.text = f'pothole_{d*100:.0f}cm'; ma.markers.append(m)
            self.pub_pot.publish(ma)

    def _cb_nonground(self, msg: PointCloud2):
        if self._surface is None or self._grid is None:
            return
        self._fc += 1
        xyz = _pc2_to_xyz(msg)
        n = len(xyz)
        if n < 20:
            return

        # residual analysis
        S_z = _sample_surface(self._surface, self._grid, xyz[:, 0], xyz[:, 1])
        residuals = xyz[:, 2] - S_z
        r = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2)
        th = _adaptive_threshold(r,
                                 self.get_parameter('residual_th_near').value,
                                 self.get_parameter('residual_th_far').value)

        # cluster flagged points
        clusters = _cluster_residual_pts(xyz, residuals, th,
                                         self.get_parameter('min_cluster_pts').value)
        if not clusters:
            return

        now = self.get_clock().now().to_msg(); fid = msg.header.frame_id
        cthr = self.get_parameter('confidence_threshold').value
        li = self.get_parameter('log_interval').value
        bh, bl = MarkerArray(), MarkerArray(); ll = []
        oi = []

        for pts_c in clusters:
            c_z = _sample_surface(self._surface, self._grid, pts_c[:, 0], pts_c[:, 1])
            tid, label, cent, dims = _classify_surface(pts_c, c_z, self._ground_z or 0.0)
            oi.append((tid, label, cent, dims, len(pts_c)))

        if not oi:
            return

        tracked = self._match_tracks(
            np.array([x[2] for x in oi]), [x[0] for x in oi], [x[1] for x in oi])

        for i, (tid, label, cent, dims, n_pts) in enumerate(oi):
            st, sl = tracked[i]
            th_hist = []
            for t in self._tracks.values():
                if t.get('_m') and np.linalg.norm(np.array(t['centroid']) - cent) < 0.1:
                    th_hist = list(t['th']); break
            v = 0.0
            # rough verticality from PCA (recompute if needed)
            conf = _confidence_surface(n_pts, v, 0.0, th_hist)
            hi = conf >= cthr

            if self._fc % li == 0:
                ll.append(f'[{TYPE_LABELS[st]}] N={n_pts} c={conf:.2f} H={dims[2]:.2f}m → {sl}')

            box = Marker(header=Header(frame_id=fid, stamp=now), ns=TYPE_LABELS[st], id=i,
                         type=Marker.CUBE, action=Marker.ADD)
            box.pose.position.x, box.pose.position.y, box.pose.position.z = float(cent[0]), float(cent[1]), float(cent[2])
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = float(dims[0]), float(dims[1]), float(dims[2])
            box.color = _mk_color(st, 0.25 if not hi else None); box.lifetime.nanosec = 300_000_000
            (bh if hi else bl).markers.append(box)

        self.pub_boxes.publish(bh)
        if bl.markers:
            self.pub_low.publish(bl)
        if ll and self._fc % li == 0:
            self.get_logger().info(f'F{self._fc}: ' + ' | '.join(ll))


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
