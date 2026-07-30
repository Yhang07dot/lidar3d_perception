#!/usr/bin/env python3
"""
PointCloud2 range & height filter.

Filters points by:
  - Distance from sensor: min_range ≤ sqrt(x²+y²+z²) ≤ max_range
  - Height (Z in laser_link frame): min_height ≤ z ≤ max_height
  - Horizontal angle (2026-07-30): |arctan2(y,x)| ≤ angle_limit_deg
    (±135° keeps front+sides, discards rear 90°. 180°=no filter. Tune via ros2 param set)

Publishes filtered PointCloud2 to /cx/lslidar_point_cloud_filtered.

All thresholds are ROS2 parameters (adjustable at runtime via ros2 param set or launch args).

Tuning guide (2026-07-30):
  max_range: 16线LiDAR建议≤25m。50m时束间距~1.75m,PCA不可靠→幽灵障碍物
  angle_limit_deg: 车用=135(去后方)。若需全向感知设为180
  confidence_threshold (cluster_analyzer): 坡闪烁→降低(0.35), 噪点多→升高(0.6)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy  # 2026-07-29: QoS for Gazebo bridge
from sensor_msgs.msg import PointCloud2


class PointCloudFilter(Node):
    """Distance + height filter for PointCloud2."""

    def __init__(self):
        super().__init__('pointcloud_filter')

        # ——— Tunable parameters ———
        # 2026-07-30: distance/height/angle filters. All runtime-tunable via ros2 param set.
        self.declare_parameter('max_range', 25.0)      # max distance (m). 16-line: ≤25 recommended
        self.declare_parameter('min_range', 0.1)       # min distance (m), removes self-hits
        self.declare_parameter('min_height', -3.0)     # min Z (m), negative = below sensor
        self.declare_parameter('max_height', 5.0)      # max Z (m)
        self.declare_parameter('angle_limit_deg', 135.0)  # half-angle (deg). 135=±135°(front+sides). 180=360°(all)

        # Read initial values
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_limit_deg = self.get_parameter('angle_limit_deg').value  # 2026-07-30

        # Watch for parameter changes at runtime
        self.add_on_set_parameters_callback(self._on_param_change)

        self.sub = self.create_subscription(
            PointCloud2,
            'input_cloud',  # remappable topic name
            self._callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),  # 2026-07-29: compat with Gazebo bridge
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/cx/lslidar_point_cloud_filtered',
            10,
        )

        self._frame_count = 0
        self._log_interval = 30  # log stats every 30 frames (~3s)

        self.get_logger().info(
            f'Filter ready: '
            f'range=[{self.min_range:.1f}, {self.max_range:.1f}]m, '
            f'height=[{self.min_height:.1f}, {self.max_height:.1f}]m, '
            f'angle_limit={self.angle_limit_deg:.0f}deg'
        )

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'max_range':
                self.max_range = p.value
            elif p.name == 'min_range':
                self.min_range = p.value
            elif p.name == 'min_height':
                self.min_height = p.value
            elif p.name == 'max_height':
                self.max_height = p.value
            elif p.name == 'angle_limit_deg':       # 2026-07-30
                self.angle_limit_deg = p.value
        self.get_logger().info(
            f'Params updated: '
            f'range=[{self.min_range:.1f}, {self.max_range:.1f}]m, '
            f'height=[{self.min_height:.1f}, {self.max_height:.1f}]m, '
            f'angle_limit={self.angle_limit_deg:.0f}deg'
        )
        return rclpy.parameter.SetParametersResult(successful=True)

    def _callback(self, msg: PointCloud2):
        point_step = msg.point_step
        n_total = msg.width * msg.height
        if n_total == 0:
            return

        # --- Build dtype from actual PointCloud2 fields (no hardcoded layout) ---
        # Map ROS2 datatype constants → numpy dtypes
        DTYPE_MAP = {
            1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
            5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
        }
        names = [f.name for f in msg.fields]
        formats = [DTYPE_MAP[f.datatype] for f in msg.fields]
        offsets = [f.offset for f in msg.fields]

        dtype = np.dtype({
            'names': names,
            'formats': formats,
            'offsets': offsets,
            'itemsize': point_step,
        })

        points = np.frombuffer(msg.data, dtype=dtype)
        x, y, z = points['x'], points['y'], points['z']

        # --- Filter ---
        dist = np.sqrt(x**2 + y**2 + z**2)
        # 2026-07-30: horizontal angle filter — discards points behind vehicle
        # x=forward, y=left. arctan2(y,x)=0=forward, ±pi=rear
        # angle_limit_deg=135 → keep |angle|≤135° (front+sides, discard rear 90°)
        # angle_limit_deg=180 → keep all 360° (no angle filter)
        angle = np.arctan2(y, x)
        angle_limit_rad = np.radians(self.angle_limit_deg)
        mask = (
            (dist >= self.min_range) & (dist <= self.max_range) &
            (z >= self.min_height) & (z <= self.max_height) &
            (np.abs(angle) <= angle_limit_rad)
        )
        keep_idx = np.where(mask)[0]

        n_kept = len(keep_idx)

        # --- Log stats periodically ---
        self._frame_count += 1
        if self._frame_count % self._log_interval == 0:
            pct = 100.0 * n_kept / n_total
            self.get_logger().info(
                f'Frame {self._frame_count}: '
                f'{n_total} → {n_kept} points ({pct:.1f}% kept)'
            )

        # --- Build filtered byte buffer ---
        if n_kept == 0:
            return

        raw = msg.data
        filtered = bytearray(n_kept * point_step)
        for i, idx in enumerate(keep_idx):
            start = idx * point_step
            filtered[i * point_step:(i + 1) * point_step] = raw[start:start + point_step]

        # --- Publish ---
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n_kept
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = point_step
        out.row_step = n_kept * point_step
        out.is_dense = msg.is_dense
        out.data = bytes(filtered)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
