#!/usr/bin/env python3
"""
Road boundary detector for continuous boundary extraction.

Subscribes:
  /patchworkpp/ground (PointCloud2)
  /obstacles/boxes_3d_surface (MarkerArray) - for boundary markers

Publishes:
  /road_boundaries/left (Path) - left boundary points
  /road_boundaries/right (Path) - right boundary points
  /road_boundaries/markers (MarkerArray) - visualization

Algorithm:
  1. 从ground点云中提取边缘点（Z突变 > 阈值）
  2. 极坐标分bin，每个角度扇区找最远连续地面点
  3. 左右分离 + 平滑 + 输出为Path消息

针对场景：一侧轮胎堆（稀疏）、一侧土坎（连续高度差）
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({'names': names, 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


class BoundaryDetector(Node):
    def __init__(self):
        super().__init__('boundary_detector')

        # ==== 参数 ====
        self.declare_parameter('angular_resolution', 1.0)    # 角度分bin分辨率（度）
        self.declare_parameter('gap_threshold', 1.2)         # 间隙阈值（m），超过认为断开
        self.declare_parameter('height_diff_threshold', 0.12) # 高度差阈值（m），路沿检测
        self.declare_parameter('smoothing_window', 7)        # 平滑窗口
        self.declare_parameter('min_boundary_points', 10)    # 最少边界点数
        self.declare_parameter('max_range', 20.0)            # 最大检测距离

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, '/patchworkpp/ground', self._callback, qos)

        self.pub_left = self.create_publisher(Path, '/road_boundaries/left', 10)
        self.pub_right = self.create_publisher(Path, '/road_boundaries/right', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/road_boundaries/markers', 10)

        self._frame_count = 0
        self.get_logger().info('Boundary Detector ready')

    def _callback(self, msg: PointCloud2):
        self._frame_count += 1
        xyz = _pc2_to_xyz(msg)
        if len(xyz) < 50:
            return

        # 参数
        ang_res = self.get_parameter('angular_resolution').value
        gap_thr = self.get_parameter('gap_threshold').value
        h_diff_thr = self.get_parameter('height_diff_threshold').value
        smooth_win = self.get_parameter('smoothing_window').value
        min_pts = self.get_parameter('min_boundary_points').value
        max_r = self.get_parameter('max_range').value

        # 极坐标分bin
        r = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2)
        th = np.arctan2(xyz[:, 1], xyz[:, 0])
        z = xyz[:, 2]

        mask = (r > 0.5) & (r < max_r)
        r, th, z, xyz = r[mask], th[mask], z[mask], xyz[mask]

        n_bins = int(360 / ang_res)
        th_bins = np.linspace(-math.pi, math.pi, n_bins + 1)
        bin_idx = np.digitize(th, th_bins) - 1
        bin_idx = np.clip(bin_idx, 0, n_bins - 1)

        # 每个bin找边界候选点
        boundary_candidates = []

        for bi in range(n_bins):
            mask_bin = (bin_idx == bi)
            if mask_bin.sum() < 3:
                continue

            pts_bin = xyz[mask_bin]
            r_bin = r[mask_bin]
            z_bin = z[mask_bin]

            # 按距离排序
            sorted_idx = np.argsort(r_bin)
            r_sorted = r_bin[sorted_idx]
            z_sorted = z_bin[sorted_idx]
            pts_sorted = pts_bin[sorted_idx]

            # 找最大连续段（间隙<gap_thr）
            segments = []
            seg_start = 0
            for i in range(1, len(r_sorted)):
                if r_sorted[i] - r_sorted[i-1] > gap_thr:
                    if i - seg_start >= 3:
                        segments.append((seg_start, i))
                    seg_start = i
            if len(r_sorted) - seg_start >= 3:
                segments.append((seg_start, len(r_sorted)))

            # 取最长段的终点作为边界候选
            if segments:
                longest_seg = max(segments, key=lambda s: s[1] - s[0])
                end_idx = longest_seg[1] - 1

                # 检查终点是否有高度突变（路沿特征）
                if end_idx >= 2:
                    z_grad = abs(z_sorted[end_idx] - z_sorted[end_idx-2])
                    if z_grad > h_diff_thr:
                        boundary_candidates.append(pts_sorted[end_idx])
                    else:
                        # 没有高度突变，但是最远点也可能是边界
                        boundary_candidates.append(pts_sorted[end_idx])

        if len(boundary_candidates) < min_pts:
            return

        boundary_pts = np.array(boundary_candidates)

        # 左右分离（基于Y坐标）
        left_mask = boundary_pts[:, 1] > 0
        right_mask = boundary_pts[:, 1] < 0

        left_pts = boundary_pts[left_mask]
        right_pts = boundary_pts[right_mask]

        # 平滑
        def smooth_boundary(pts, window):
            if len(pts) < window:
                return pts
            # 按X排序
            sorted_idx = np.argsort(pts[:, 0])
            pts_sorted = pts[sorted_idx]
            smoothed = pts_sorted.copy()
            for i in range(len(pts_sorted)):
                start = max(0, i - window // 2)
                end = min(len(pts_sorted), i + window // 2 + 1)
                smoothed[i] = pts_sorted[start:end].mean(axis=0)
            return smoothed

        left_smooth = smooth_boundary(left_pts, smooth_win) if len(left_pts) >= min_pts else left_pts
        right_smooth = smooth_boundary(right_pts, smooth_win) if len(right_pts) >= min_pts else right_pts

        # 发布Path消息
        now = self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id

        if len(left_smooth) >= min_pts:
            path_left = Path()
            path_left.header = Header(frame_id=frame_id, stamp=now)
            for pt in left_smooth:
                pose = PoseStamped()
                pose.header = path_left.header
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = float(pt[2])
                pose.pose.orientation.w = 1.0
                path_left.poses.append(pose)
            self.pub_left.publish(path_left)

        if len(right_smooth) >= min_pts:
            path_right = Path()
            path_right.header = Header(frame_id=frame_id, stamp=now)
            for pt in right_smooth:
                pose = PoseStamped()
                pose.header = path_right.header
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = float(pt[2])
                pose.pose.orientation.w = 1.0
                path_right.poses.append(pose)
            self.pub_right.publish(path_right)

        # 可视化
        ma = MarkerArray()
        if len(left_smooth) >= min_pts:
            m = Marker(header=Header(frame_id=frame_id, stamp=now),
                       ns='left_boundary', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
            for pt in left_smooth:
                from geometry_msgs.msg import Point
                m.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
            m.scale.x = 0.1
            m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            m.lifetime.nanosec = 500_000_000
            ma.markers.append(m)

        if len(right_smooth) >= min_pts:
            m = Marker(header=Header(frame_id=frame_id, stamp=now),
                       ns='right_boundary', id=1, type=Marker.LINE_STRIP, action=Marker.ADD)
            for pt in right_smooth:
                from geometry_msgs.msg import Point
                m.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
            m.scale.x = 0.1
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8)
            m.lifetime.nanosec = 500_000_000
            ma.markers.append(m)

        if ma.markers:
            self.pub_markers.publish(ma)

        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: Left={len(left_smooth)} pts, Right={len(right_smooth)} pts')


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
