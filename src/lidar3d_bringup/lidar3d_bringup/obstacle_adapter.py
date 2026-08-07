#!/usr/bin/env python3
"""Adapt surface classifications into stable planning obstacles.

The adapter publishes the existing two-class planning interface:

* ``tall``: static, impassable obstacle tracked in ``world_frame``.
* ``flat_ground``: passable terrain from the current perception frame only.

Static tall tracks are stored in the map frame and reprojected into
``target_frame`` for every input frame. They are removed only after the
vehicle has passed them, never because the LiDAR temporarily returns no points.
"""

from dataclasses import dataclass
import math
import re
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


TYPE_COLORS = {
    'tall': (1.0, 0.0, 0.0, 0.7),
    'flat_ground': (0.0, 0.8, 0.0, 0.5),
}

CONFIDENCE_PATTERN = re.compile(r'(?:^|\s)c=([0-9]+(?:\.[0-9]+)?)\b')
SLOPE_METADATA_PATTERN = re.compile(
    r'^(passable_slope\s+)'
    r'apex_x=(?P<x>-?[0-9]+(?:\.[0-9]+)?)\s+'
    r'apex_y=(?P<y>-?[0-9]+(?:\.[0-9]+)?)\s+'
    r'apex_z=(?P<z>-?[0-9]+(?:\.[0-9]+)?)'
    r'(?P<tail>.*)$'
)


@dataclass
class StaticTallTrack:
    """A static tall obstacle represented in the map frame."""

    track_id: int
    world_xyz: Tuple[float, float, float]
    scale_xyz: Tuple[float, float, float]
    hit_count: int
    last_observation_ns: int


def simplify_classification(surface_ns: str) -> str:
    """Map the four surface-detector classes to the control interface."""
    if surface_ns == 'obstacle':
        return 'tall'
    return 'flat_ground'


class ObstacleAdapter(Node):
    """Publish current flat terrain and map-frame static tall tracks."""

    def __init__(self):
        super().__init__('obstacle_adapter')

        self.declare_parameter('source_frame', 'laser_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('input_topic', '/obstacles/boxes')
        self.declare_parameter('passthrough', False)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('road_boundary_topic', '/road_boundary_markers')
        self.declare_parameter('boundary_exclusion_distance_m', 1.0)
        self.declare_parameter('track_create_confidence', 0.80)
        self.declare_parameter('track_update_confidence', 0.60)
        self.declare_parameter('track_association_distance_m', 1.5)
        self.declare_parameter('track_position_alpha', 0.30)
        self.declare_parameter('track_scale_alpha', 0.30)
        self.declare_parameter('track_release_behind_m', 2.0)

        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.input_topic = self.get_parameter('input_topic').value
        self.passthrough = self.get_parameter('passthrough').value
        self.world_frame = self.get_parameter('world_frame').value
        self.road_boundary_topic = self.get_parameter('road_boundary_topic').value
        self.boundary_exclusion_distance = float(
            self.get_parameter('boundary_exclusion_distance_m').value)
        self.create_confidence = float(
            self.get_parameter('track_create_confidence').value)
        self.update_confidence = float(
            self.get_parameter('track_update_confidence').value)
        self.association_distance = float(
            self.get_parameter('track_association_distance_m').value)
        self.position_alpha = float(
            self.get_parameter('track_position_alpha').value)
        self.scale_alpha = float(self.get_parameter('track_scale_alpha').value)
        self.release_behind = float(
            self.get_parameter('track_release_behind_m').value)

        if not 0.0 <= self.update_confidence <= self.create_confidence <= 1.0:
            raise ValueError(
                'Expected 0 <= track_update_confidence '
                '<= track_create_confidence <= 1')
        if self.association_distance <= 0.0:
            raise ValueError('track_association_distance_m must be positive')
        if self.boundary_exclusion_distance < 0.0:
            raise ValueError('boundary_exclusion_distance_m must be non-negative')
        if not 0.0 < self.position_alpha <= 1.0:
            raise ValueError('track_position_alpha must be in (0, 1]')
        if not 0.0 < self.scale_alpha <= 1.0:
            raise ValueError('track_scale_alpha must be in (0, 1]')
        if self.release_behind < 0.0:
            raise ValueError('track_release_behind_m must be non-negative')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tracks: Dict[int, StaticTallTrack] = {}
        self._boundary_lines_world: Dict[str, List[Tuple[float, float]]] = {}
        self._next_track_id = 1
        self._frame_count = 0

        self.sub = self.create_subscription(
            MarkerArray,
            self.input_topic,
            self.callback,
            10,
        )
        self.boundary_sub = self.create_subscription(
            MarkerArray,
            self.road_boundary_topic,
            self._on_road_boundaries,
            10,
        )
        self.pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self.get_logger().info(
            f'Obstacle Adapter ready — {self.input_topic} → /obstacle_markers '
            f'(static tall tracks in {self.world_frame}; '
            f'create c>={self.create_confidence:.2f}, '
            f'update c>={self.update_confidence:.2f}, '
            f'boundary exclusion={self.boundary_exclusion_distance:.2f}m, '
            f'release x<-{self.release_behind:.2f}m)')

    def _lookup(self, target: str, source: str) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    @staticmethod
    def _apply_transform(
            transform: TransformStamped,
            point_x: float,
            point_y: float,
            point_z: float) -> Tuple[float, float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion_w = rotation.w
        quaternion_x = rotation.x
        quaternion_y = rotation.y
        quaternion_z = rotation.z

        cross_x = 2.0 * (quaternion_y * point_z - quaternion_z * point_y)
        cross_y = 2.0 * (quaternion_z * point_x - quaternion_x * point_z)
        cross_z = 2.0 * (quaternion_x * point_y - quaternion_y * point_x)
        rotated_x = point_x + quaternion_w * cross_x + (
            quaternion_y * cross_z - quaternion_z * cross_y)
        rotated_y = point_y + quaternion_w * cross_y + (
            quaternion_z * cross_x - quaternion_x * cross_z)
        rotated_z = point_z + quaternion_w * cross_z + (
            quaternion_x * cross_y - quaternion_y * cross_x)
        return (
            rotated_x + translation.x,
            rotated_y + translation.y,
            rotated_z + translation.z,
        )

    @staticmethod
    def _confidence(marker: Marker) -> Optional[float]:
        match = CONFIDENCE_PATTERN.search(marker.text)
        return float(match.group(1)) if match else None

    @staticmethod
    def _blend(
            previous: Tuple[float, float, float],
            observed: Tuple[float, float, float],
            alpha: float) -> Tuple[float, float, float]:
        return tuple(
            (1.0 - alpha) * previous_value + alpha * observed_value
            for previous_value, observed_value in zip(previous, observed))

    def _nearest_track(
            self, world_xyz: Tuple[float, float, float]) -> Optional[StaticTallTrack]:
        nearest = None
        nearest_distance = self.association_distance
        for track in self._tracks.values():
            distance = math.hypot(
                track.world_xyz[0] - world_xyz[0],
                track.world_xyz[1] - world_xyz[1])
            if distance <= nearest_distance:
                nearest = track
                nearest_distance = distance
        return nearest

    @staticmethod
    def _has_obstacle_semantic(marker: Marker) -> bool:
        return marker.text.startswith('obstacle_')

    def _boundary_y_at_x(
            self,
            side: str,
            base_x: float,
            base_from_world: TransformStamped) -> Optional[float]:
        line = self._boundary_lines_world.get(side)
        if not line:
            return None

        y_values = []
        for index in range(len(line) - 1):
            start_x, start_y, _ = self._apply_transform(
                base_from_world, line[index][0], line[index][1], 0.0)
            end_x, end_y, _ = self._apply_transform(
                base_from_world, line[index + 1][0], line[index + 1][1], 0.0)
            delta_x = end_x - start_x
            if abs(delta_x) < 1e-6:
                continue
            if not min(start_x, end_x) <= base_x <= max(start_x, end_x):
                continue
            ratio = (base_x - start_x) / delta_x
            y_values.append(start_y + ratio * (end_y - start_y))

        if not y_values:
            return None
        return min(y_values) if side == 'road_left' else max(y_values)

    def _is_roadside_object(
            self,
            world_xyz: Tuple[float, float, float],
            base_from_world: TransformStamped) -> bool:
        base_x, base_y, _ = self._apply_transform(base_from_world, *world_xyz)
        left_y = self._boundary_y_at_x('road_left', base_x, base_from_world)
        right_y = self._boundary_y_at_x('road_right', base_x, base_from_world)

        if left_y is not None and right_y is not None:
            if left_y <= right_y:
                return False
            return not right_y < base_y < left_y

        if left_y is not None:
            return (
                base_y >= left_y
                and base_y - left_y <= self.boundary_exclusion_distance
            )
        if right_y is not None:
            return (
                base_y <= right_y
                and right_y - base_y <= self.boundary_exclusion_distance
            )
        return False

    def _on_road_boundaries(self, msg: MarkerArray) -> None:
        updated_lines: Dict[str, List[Tuple[float, float]]] = {}
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP:
                continue
            if marker.ns not in ('road_left', 'road_right') or len(marker.points) < 2:
                continue

            source_frame = marker.header.frame_id or self.target_frame
            if source_frame == self.world_frame:
                world_from_source = None
            else:
                world_from_source = self._lookup(self.world_frame, source_frame)
                if world_from_source is None:
                    self.get_logger().warn(
                        f'TF lookup failed ({source_frame}→{self.world_frame}) '
                        'for road boundary',
                        throttle_duration_sec=5.0)
                    continue

            line = []
            for point in marker.points:
                source_x = marker.pose.position.x + point.x
                source_y = marker.pose.position.y + point.y
                source_z = marker.pose.position.z + point.z
                if world_from_source is None:
                    world_x, world_y = source_x, source_y
                else:
                    world_x, world_y, _ = self._apply_transform(
                        world_from_source, source_x, source_y, source_z)
                line.append((world_x, world_y))
            updated_lines[marker.ns] = line

        self._boundary_lines_world.update(updated_lines)

    def _update_or_create_track(
            self,
            world_xyz: Tuple[float, float, float],
            scale_xyz: Tuple[float, float, float],
            confidence: float,
            now_ns: int) -> None:
        track = self._nearest_track(world_xyz)
        if track is None:
            if confidence < self.create_confidence:
                return
            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = StaticTallTrack(
                track_id=track_id,
                world_xyz=world_xyz,
                scale_xyz=scale_xyz,
                hit_count=1,
                last_observation_ns=now_ns,
            )
            return

        if confidence < self.update_confidence:
            return
        track.world_xyz = self._blend(
            track.world_xyz, world_xyz, self.position_alpha)
        track.scale_xyz = self._blend(
            track.scale_xyz, scale_xyz, self.scale_alpha)
        track.hit_count += 1
        track.last_observation_ns = now_ns

    def _remove_passed_tracks(self, base_from_world: TransformStamped) -> None:
        passed_track_ids = []
        for track_id, track in self._tracks.items():
            base_x, _, _ = self._apply_transform(
                base_from_world, *track.world_xyz)
            if base_x < -self.release_behind:
                passed_track_ids.append(track_id)
        for track_id in passed_track_ids:
            del self._tracks[track_id]

    def _make_marker(
            self,
            label: str,
            marker_id: int,
            base_xyz: Tuple[float, float, float],
            scale_xyz: Tuple[float, float, float],
            stamp,
            text: str = '') -> Marker:
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = stamp
        marker.ns = label
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = base_xyz[0]
        marker.pose.position.y = base_xyz[1]
        marker.pose.position.z = base_xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_xyz[0]
        marker.scale.y = scale_xyz[1]
        marker.scale.z = scale_xyz[2]
        red, green, blue, alpha = TYPE_COLORS[label]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        marker.lifetime.nanosec = 200_000_000
        marker.text = text
        return marker

    def _transform_slope_metadata(
            self,
            source_text: str,
            base_from_sensor: TransformStamped) -> str:
        """Rewrite slope apex coordinates from the source frame into base_link."""
        match = SLOPE_METADATA_PATTERN.match(source_text)
        if match is None:
            return source_text

        apex_xyz = self._apply_transform(
            base_from_sensor,
            float(match.group('x')),
            float(match.group('y')),
            float(match.group('z')))
        return (
            f'{match.group(1)}'
            f'apex_x={apex_xyz[0]:.2f} '
            f'apex_y={apex_xyz[1]:.2f} '
            f'apex_z={apex_xyz[2]:.2f}'
            f'{match.group("tail")}')

    def callback(self, msg: MarkerArray):
        self._frame_count += 1
        now_ns = self.get_clock().now().nanoseconds
        now = self.get_clock().now().to_msg()

        base_from_sensor = self._lookup(self.target_frame, self.source_frame)
        if base_from_sensor is None:
            self.get_logger().warn(
                'TF lookup failed (sensor→base_link)',
                throttle_duration_sec=5.0)
            return

        world_from_base = self._lookup(self.world_frame, self.target_frame)
        base_from_world = self._lookup(self.target_frame, self.world_frame)
        can_track = world_from_base is not None and base_from_world is not None
        if not can_track:
            self.get_logger().warn(
                f'TF {self.world_frame}↔{self.target_frame} unavailable; '
                'publishing current high-confidence tall observations only',
                throttle_duration_sec=5.0)

        current_flats: List[Tuple[
            int,
            Tuple[float, float, float],
            Tuple[float, float, float],
            str,
            bool,
        ]] = []
        fallback_talls: List[Tuple[int, Tuple[float, float, float], Tuple[float, float, float]]] = []

        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.type != Marker.CUBE:
                continue

            label = simplify_classification(marker.ns) if self.passthrough else 'flat_ground'
            base_xyz = self._apply_transform(
                base_from_sensor,
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z)
            scale_xyz = (marker.scale.x, marker.scale.y, marker.scale.z)

            if label == 'flat_ground':
                is_slope = marker.text.startswith('passable_slope apex_x=')
                flat_text = marker.text
                if is_slope:
                    flat_text = self._transform_slope_metadata(
                        marker.text, base_from_sensor)
                current_flats.append(
                    (marker.id, base_xyz, scale_xyz, flat_text, is_slope))
                continue

            confidence = self._confidence(marker)
            if confidence is None:
                self.get_logger().warn(
                    'Skipping tall candidate without c=<score> in marker.text',
                    throttle_duration_sec=5.0)
                continue
            if not self._has_obstacle_semantic(marker):
                continue

            if not can_track:
                if confidence >= self.create_confidence:
                    fallback_talls.append((marker.id, base_xyz, scale_xyz))
                continue

            world_xyz = self._apply_transform(world_from_base, *base_xyz)
            if self._is_roadside_object(world_xyz, base_from_world):
                continue
            self._update_or_create_track(
                world_xyz, scale_xyz, confidence, now_ns)

        output = MarkerArray()
        published_tall_positions: List[Tuple[float, float]] = []

        if can_track:
            self._remove_passed_tracks(base_from_world)
            for track_id in sorted(self._tracks):
                track = self._tracks[track_id]
                base_xyz = self._apply_transform(base_from_world, *track.world_xyz)
                output.markers.append(self._make_marker(
                    'tall', track.track_id, base_xyz, track.scale_xyz, now))
                published_tall_positions.append((base_xyz[0], base_xyz[1]))

        for marker_id, base_xyz, scale_xyz in fallback_talls:
            output.markers.append(self._make_marker(
                'tall', marker_id, base_xyz, scale_xyz, now))
            published_tall_positions.append((base_xyz[0], base_xyz[1]))

        for marker_id, base_xyz, scale_xyz, text, is_slope in current_flats:
            overlaps_tall = any(
                math.hypot(base_xyz[0] - tall_x, base_xyz[1] - tall_y)
                <= self.association_distance
                for tall_x, tall_y in published_tall_positions)
            # A slope is terrain metadata for the controller, not a competing
            # collision object.  Keep it even when a tall obstacle stands on it.
            if is_slope or not overlaps_tall:
                output.markers.append(self._make_marker(
                    'flat_ground', marker_id, base_xyz, scale_xyz, now, text))

        self.pub.publish(output)

        if self._frame_count % 10 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: tracks={len(self._tracks)}, '
                f'boundaries={len(self._boundary_lines_world)}, '
                f'published={len(output.markers)}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
