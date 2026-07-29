#!/usr/bin/env python3
"""
3D Euclidean clustering via voxel grid + 26-neighbour flood fill.

Subscribes to /patchworkpp/nonground (PointCloud2), publishes:
  /clusters/points_3d   — PointCloud2 with intensity=cluster_id
  /clusters/markers_3d  — MarkerArray with SPHERE centroids

Pure numpy — no PCL/open3d dependency.

Added 2026-07-29: parallel 3D clustering pipeline for slope-obstacle discrimination.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# PointCloud2 field dtype mapping (same as pointcloud_filter.py)
DTYPE_MAP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N,3) xyz array from PointCloud2 message."""
    names = [f.name for f in msg.fields]
    formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtype = np.dtype({
        'names': names, 'formats': formats,
        'offsets': offsets, 'itemsize': msg.point_step,
    })
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([points['x'], points['y'], points['z']])


def _xyz_to_pc2(xyz: np.ndarray, header, cluster_ids: np.ndarray) -> PointCloud2:
    """Build PointCloud2 with x,y,z and intensity=cluster_id."""
    n = len(xyz)
    out = PointCloud2()
    out.header = header
    out.height = 1
    out.width = n
    out.is_bigendian = False
    out.is_dense = True
    out.point_step = 16  # x(float32) + y(float32) + z(float32) + intensity(float32)
    out.row_step = out.point_step * n

    out.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    data = np.zeros(n, dtype=[
        ('x', np.float32), ('y', np.float32),
        ('z', np.float32), ('intensity', np.float32),
    ])
    data['x'] = xyz[:, 0].astype(np.float32)
    data['y'] = xyz[:, 1].astype(np.float32)
    data['z'] = xyz[:, 2].astype(np.float32)
    data['intensity'] = cluster_ids.astype(np.float32)
    out.data = data.tobytes()
    return out


def _voxel_cluster(
    xyz: np.ndarray,
    voxel_size: float = 0.15,
    tolerance: float = 0.5,
    min_points: int = 5,
) -> np.ndarray:
    """3D voxel-grid flood-fill clustering.  Returns cluster_id per point (0=noise)."""
    n = len(xyz)
    if n == 0:
        return np.zeros(0, dtype=np.int32)

    # --- voxelise ---
    voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)
    # shift to non-negative
    vmin = voxel_indices.min(axis=0)
    voxel_indices -= vmin

    # unique voxels
    voxel_keys, inverse = np.unique(
        voxel_indices[:, 0] * 100000 + voxel_indices[:, 1] * 1000 + voxel_indices[:, 2],
        return_inverse=True,
    )
    n_voxels = len(voxel_keys)

    # voxel → point indices mapping
    voxel_point_indices = [[] for _ in range(n_voxels)]
    for pi, vi in enumerate(inverse):
        voxel_point_indices[vi].append(pi)

    # Reconstruct voxel coordinates from inverse mapping
    vx_coords = np.zeros((n_voxels, 3), dtype=np.int32)
    for vi in range(n_voxels):
        vx_coords[vi] = voxel_indices[inverse == vi][0]

    # Build sparse grid dict: (ix, iy, iz) → voxel_id
    grid = {}
    for vi in range(n_voxels):
        grid[tuple(vx_coords[vi])] = vi

    # --- flood fill ---
    # 26-neighbour offsets
    offsets = np.array([
        [dx, dy, dz]
        for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ], dtype=np.int32)

    visited = np.zeros(n_voxels, dtype=bool)
    cluster_id_counter = 0
    point_cluster_ids = np.zeros(n, dtype=np.int32)

    for vi in range(n_voxels):
        if visited[vi]:
            continue
        visited[vi] = True

        # start new cluster
        cluster_id_counter += 1
        frontier = [vi]
        cluster_voxels = [vi]

        while frontier:
            current = frontier.pop()
            current_coord = vx_coords[current]

            for offset in offsets:
                neighbour_coord = tuple(current_coord + offset)
                nbr = grid.get(neighbour_coord)
                if nbr is not None and not visited[nbr]:
                    visited[nbr] = True
                    frontier.append(nbr)
                    cluster_voxels.append(nbr)

        # count total points in cluster
        total_pts = sum(len(voxel_point_indices[v]) for v in cluster_voxels)
        if total_pts < min_points:
            # reject cluster
            for v in cluster_voxels:
                for pi in voxel_point_indices[v]:
                    point_cluster_ids[pi] = 0  # noise
        else:
            for v in cluster_voxels:
                for pi in voxel_point_indices[v]:
                    point_cluster_ids[pi] = cluster_id_counter

    return point_cluster_ids


class EuclideanCluster3D(Node):
    """3D voxel-based Euclidean clustering for non-ground point clouds."""

    def __init__(self):
        super().__init__('euclidean_cluster_3d')

        self.declare_parameter('voxel_leaf_size', 0.15)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 5)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            PointCloud2, '/patchworkpp/nonground', self._callback, qos,
        )
        self.pub_points = self.create_publisher(
            PointCloud2, '/clusters/points_3d', 10,
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, '/clusters/markers_3d', 10,
        )

        self._frame_count = 0
        self._log_interval = 30

        self.get_logger().info(
            '3D Euclidean clustering ready — '
            f'voxel={self.get_parameter("voxel_leaf_size").value}m, '
            f'tolerance={self.get_parameter("cluster_tolerance").value}m'
        )

    def _callback(self, msg: PointCloud2):
        self._frame_count += 1
        xyz = _pc2_to_xyz(msg)
        n_total = len(xyz)

        if n_total == 0:
            return

        voxel_size = self.get_parameter('voxel_leaf_size').value
        tolerance = self.get_parameter('cluster_tolerance').value
        min_pts = self.get_parameter('min_cluster_size').value

        cluster_ids = _voxel_cluster(xyz, voxel_size, tolerance, min_pts)

        n_clustered = int((cluster_ids > 0).sum())
        n_clusters = int(cluster_ids.max())

        if self._frame_count % self._log_interval == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: {n_total} pts → '
                f'{n_clusters} clusters ({n_clustered} pts clustered)'
            )

        if n_clusters == 0:
            return

        # --- publish point cloud with cluster IDs ---
        out_cloud = _xyz_to_pc2(xyz, msg.header, cluster_ids)
        self.pub_points.publish(out_cloud)

        # --- publish cluster centroids ---
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for cid in range(1, n_clusters + 1):
            mask = cluster_ids == cid
            pts = xyz[mask]
            if len(pts) == 0:
                continue
            centroid = pts.mean(axis=0)

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = now
            marker.ns = 'cluster_3d'
            marker.id = cid
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color = ColorRGBA(r=0.3, g=0.7, b=1.0, a=0.6)
            marker.lifetime.nanosec = 300_000_000

            marker_array.markers.append(marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = EuclideanCluster3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
