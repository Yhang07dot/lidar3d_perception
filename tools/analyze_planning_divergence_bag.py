#!/usr/bin/env python3
"""Summarize perception windows around planned-path divergence in a rosbag2 bag."""

import argparse
import bisect
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class PathSample:
    timestamp_ns: int
    anchor_error_m: float
    nearest_error_m: float
    path_jump_m: float
    path_points: int
    tall_count: int
    flat_count: int
    left_boundary_points: int
    right_boundary_points: int


def _read_topics(bag_path: str, topics: Dict[str, str]) -> Dict[str, list]:
    message_types = {topic: get_message(type_name) for topic, type_name in topics.items()}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    messages = {topic: [] for topic in topics}
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in message_types:
            messages[topic].append(
                (timestamp_ns, deserialize_message(data, message_types[topic]))
            )
    return messages


def _nearest(messages: Sequence[Tuple[int, object]], timestamps: Sequence[int], timestamp_ns: int):
    index = bisect.bisect_left(timestamps, timestamp_ns)
    candidates = messages[max(0, index - 1):min(len(messages), index + 1)]
    return min(candidates, key=lambda item: abs(item[0] - timestamp_ns))


def _distance(first: Tuple[float, float], second: Tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _path_jump(previous, current) -> float:
    if previous is None or not previous.poses or not current.poses:
        return 0.0
    count = min(len(previous.poses), len(current.poses), 20)
    if count == 0:
        return 0.0
    distances = []
    for index in range(count):
        previous_pose = previous.poses[index].pose.position
        current_pose = current.poses[index].pose.position
        distances.append(math.hypot(
            previous_pose.x - current_pose.x,
            previous_pose.y - current_pose.y,
        ))
    return max(distances)


def _marker_counts(marker_array) -> Tuple[int, int]:
    tall_count = sum(marker.ns == "tall" for marker in marker_array.markers)
    flat_count = sum(marker.ns == "flat_ground" for marker in marker_array.markers)
    return tall_count, flat_count


def _boundary_counts(marker_array) -> Tuple[int, int]:
    counts = {"road_left": 0, "road_right": 0}
    for marker in marker_array.markers:
        if marker.ns in counts:
            counts[marker.ns] = len(marker.points)
    return counts["road_left"], counts["road_right"]


def _event_windows(samples: Iterable[PathSample], threshold: float) -> List[List[PathSample]]:
    windows = []
    current = []
    for sample in samples:
        divergent = (
            sample.anchor_error_m > threshold
            or sample.nearest_error_m > threshold
            or sample.path_jump_m > 3.0 * threshold
        )
        if divergent:
            if current and sample.timestamp_ns - current[-1].timestamp_ns > 300_000_000:
                windows.append(current)
                current = []
            current.append(sample)
        elif current:
            windows.append(current)
            current = []
    if current:
        windows.append(current)
    return windows


def _format_report(
        bag_path: str,
        samples: Sequence[PathSample],
        threshold: float,
        frames: Counter) -> str:
    if not samples:
        return "# 规划路径诊断\n\n未找到 /planned_path 样本。\n"

    start_ns = samples[0].timestamp_ns
    anchor_max = max(sample.anchor_error_m for sample in samples)
    nearest_max = max(sample.nearest_error_m for sample in samples)
    jump_max = max(sample.path_jump_m for sample in samples)
    windows = _event_windows(samples, threshold)

    lines = [
        "# 规划路径—感知输入诊断",
        "",
        f"- bag: `{bag_path}`",
        f"- planned_path 样本: {len(samples)}",
        f"- frame_id 统计: {dict(frames)}",
        f"- 最大路径起点误差: {anchor_max:.3f} m",
        f"- 最大车辆到路径最近点误差: {nearest_max:.3f} m",
        f"- 最大相邻路径跳变: {jump_max:.3f} m",
        f"- 诊断阈值: {threshold:.3f} m",
        "",
        "## 疑似偏差窗口",
    ]
    if not windows:
        lines.append("- 未发现超过阈值的路径锚点/跳变事件。")
        return "\n".join(lines) + "\n"

    for index, window in enumerate(windows, start=1):
        first = window[0]
        last = window[-1]
        elapsed_first = (first.timestamp_ns - start_ns) / 1e9
        elapsed_last = (last.timestamp_ns - start_ns) / 1e9
        worst = max(
            window,
            key=lambda sample: max(
                sample.anchor_error_m,
                sample.nearest_error_m,
                sample.path_jump_m,
            ),
        )
        lines.extend([
            f"### 窗口 {index}: {elapsed_first:.2f}–{elapsed_last:.2f} s",
            f"- 最差样本: anchor={worst.anchor_error_m:.3f} m, "
            f"nearest={worst.nearest_error_m:.3f} m, jump={worst.path_jump_m:.3f} m",
            f"- 同步感知: tall={worst.tall_count}, flat={worst.flat_count}, "
            f"road_left={worst.left_boundary_points}, "
            f"road_right={worst.right_boundary_points}, path_points={worst.path_points}",
        ])
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=1.0,
        help="path anchor and nearest-point divergence threshold in metres",
    )
    parser.add_argument("--output", help="optional Markdown report path")
    parser.add_argument("--json-output", help="optional JSON sample output path")
    args = parser.parse_args()

    if args.threshold_m <= 0.0:
        parser.error("--threshold-m must be positive")

    topics = {
        "/planned_path": "nav_msgs/msg/Path",
        "/ground_truth/odom": "nav_msgs/msg/Odometry",
        "/obstacle_markers": "visualization_msgs/msg/MarkerArray",
        "/road_boundary_markers": "visualization_msgs/msg/MarkerArray",
    }
    messages = _read_topics(args.bag, topics)
    paths = messages["/planned_path"]
    odometry = messages["/ground_truth/odom"]
    obstacles = messages["/obstacle_markers"]
    boundaries = messages["/road_boundary_markers"]
    if not paths or not odometry:
        raise RuntimeError("bag must include /planned_path and /ground_truth/odom")

    odometry_timestamps = [timestamp for timestamp, _ in odometry]
    obstacle_timestamps = [timestamp for timestamp, _ in obstacles]
    boundary_timestamps = [timestamp for timestamp, _ in boundaries]
    samples = []
    previous_path = None
    frame_counts = Counter()
    for timestamp_ns, path in paths:
        frame_counts[path.header.frame_id] += 1
        _, odom = _nearest(odometry, odometry_timestamps, timestamp_ns)
        position = odom.pose.pose.position
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        if points:
            anchor_error = _distance((position.x, position.y), points[0])
            nearest_error = min(
                _distance((position.x, position.y), point) for point in points
            )
        else:
            anchor_error = float("inf")
            nearest_error = float("inf")

        tall_count = flat_count = left_count = right_count = 0
        if obstacles:
            _, marker_array = _nearest(obstacles, obstacle_timestamps, timestamp_ns)
            tall_count, flat_count = _marker_counts(marker_array)
        if boundaries:
            _, marker_array = _nearest(boundaries, boundary_timestamps, timestamp_ns)
            left_count, right_count = _boundary_counts(marker_array)

        samples.append(PathSample(
            timestamp_ns=timestamp_ns,
            anchor_error_m=anchor_error,
            nearest_error_m=nearest_error,
            path_jump_m=_path_jump(previous_path, path),
            path_points=len(path.poses),
            tall_count=tall_count,
            flat_count=flat_count,
            left_boundary_points=left_count,
            right_boundary_points=right_count,
        ))
        previous_path = path

    report = _format_report(args.bag, samples, args.threshold_m, frame_counts)
    print(report, end="")
    if args.output:
        Path(args.output).write_text(report, encoding="utf-8")
    if args.json_output:
        Path(args.json_output).write_text(
            json.dumps([asdict(sample) for sample in samples], ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
