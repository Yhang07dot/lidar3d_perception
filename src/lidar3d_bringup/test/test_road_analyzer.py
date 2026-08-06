"""Unit tests for longitudinal road-boundary extraction."""

import numpy as np

from lidar3d_bringup.road_analyzer import (
    _extract_lane_boundaries,
    _merge_live_with_cache,
)


def _extract(points):
    return _extract_lane_boundaries(
        np.asarray(points, dtype=float),
        min_forward=1.0,
        max_forward=20.0,
        min_lateral=0.75,
        forward_bin_size=0.5,
        min_road_width=3.0,
        max_road_width=12.0,
        width_tolerance=1.0,
        max_lateral_step=1.0,
        min_points_per_side=2,
    )


def test_extracts_two_forward_ordered_obstacle_boundaries():
    """Keeps paired roadside obstacles and rejects a central outlier."""
    points = []
    for x_value in np.arange(1.1, 15.0, 0.5):
        points.extend([
            [x_value, 4.0, 0.2],
            [x_value + 0.05, 4.1, 0.3],
            [x_value + 0.10, 4.2, 0.4],
            [x_value, -4.0, 0.2],
            [x_value + 0.05, -4.1, 0.3],
            [x_value + 0.10, -4.2, 0.4],
        ])
    points.extend([
        [7.10, 1.0, 0.2],
        [7.15, 1.1, 0.3],
        [7.20, 1.2, 0.4],
    ])

    left, right = _extract(points)

    assert len(left) >= 25
    assert len(right) == len(left)
    assert np.all(np.diff(left[:, 0]) > 0.0)
    assert np.all(np.diff(right[:, 0]) > 0.0)
    assert np.all(left[:, 1] > 3.5)
    assert np.all(right[:, 1] < -3.5)


def test_live_samples_override_overlapping_cache_bins():
    """Keeps live samples and uses cache only to fill missing bins."""
    live = np.asarray([[4.1, 4.0], [4.6, 4.1], [5.1, 4.0]])
    cached = np.asarray([[3.9, 4.0], [4.4, 4.2], [4.9, 4.0]])

    merged = _merge_live_with_cache(live, cached, 0.5, 3)

    assert len(merged) == 4
    assert np.all(np.diff(merged[:, 0]) > 0.0)
    assert np.allclose(merged[1:, 0], [4.1, 4.6, 5.1])
    assert np.allclose(merged[:, 1], 4.0, atol=0.15)


def test_keeps_width_consistent_pairs_across_sparse_x_bins():
    """Retains lane support when 16-line returns are separated longitudinally."""
    points = []
    for x_value in (4.1, 4.6, 9.1, 9.6, 14.1, 14.6):
        points.extend([
            [x_value, 4.0, 0.2],
            [x_value + 0.05, 4.1, 0.3],
            [x_value, -4.0, 0.2],
            [x_value + 0.05, -4.1, 0.3],
        ])

    left, right = _extract(points)

    assert len(left) == 6
    assert len(right) == 6
    assert np.all(np.diff(left[:, 0]) > 0.0)
