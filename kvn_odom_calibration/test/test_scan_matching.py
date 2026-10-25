import math

import numpy as np
import pytest

from kvn_odom_calibration.scan_matching import (
    icp, scan_to_points, se2, se2_params, transform_points, voxel_downsample)


def room_scan(pose, noise=0.0, seed=0):
    """Simulated 360-degree scan from pose (x, y, yaw) in a 6x4 m room with a pillar."""
    rng = np.random.default_rng(seed)
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    walls = [((-3, -2), (3, -2)), ((3, -2), (3, 2)), ((3, 2), (-3, 2)), ((-3, 2), (-3, -2)),
             ((1.0, 0.5), (1.4, 0.5)), ((1.4, 0.5), (1.4, 0.9)), ((1.4, 0.9), (1.0, 0.9)),
             ((1.0, 0.9), (1.0, 0.5)), ((-2.0, -1.0), (-1.0, -1.6))]
    x0, y0, yaw = pose
    ranges = np.full(angles.size, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a + yaw), math.sin(a + yaw)])
        for (ax, ay), (bx, by) in walls:
            e = np.array([bx - ax, by - ay])
            m = np.array([[d[0], -e[0]], [d[1], -e[1]]])
            if abs(np.linalg.det(m)) < 1e-12:
                continue
            t, s = np.linalg.solve(m, np.array([ax - x0, ay - y0]))
            if t > 0 and 0 <= s <= 1:
                ranges[i] = min(ranges[i], t)
    ranges += rng.normal(0.0, noise, ranges.size)
    return scan_to_points(ranges, -math.pi, 2 * math.pi / angles.size, 0.05, 12.0)


@pytest.mark.parametrize('motion,guess', [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((1.0, 0.03, 0.02), (0.9, 0.0, 0.0)),
    ((0.0, 0.0, 0.3), (0.0, 0.0, 0.15)),
    ((-0.8, -0.05, -0.05), (-0.7, 0.0, 0.0)),
])
def test_icp_recovers_motion(motion, guess):
    start = voxel_downsample(room_scan((0.0, 0.0, 0.0), noise=0.005, seed=1), 0.03)
    end = voxel_downsample(room_scan(motion, noise=0.005, seed=2), 0.03)
    result = icp(end, start, initial=se2(*guess))
    x, y, yaw = se2_params(result.transform)
    assert x == pytest.approx(motion[0], abs=0.005)
    assert y == pytest.approx(motion[1], abs=0.005)
    assert yaw == pytest.approx(motion[2], abs=math.radians(0.2))
    assert result.inlier_ratio > 0.9
    assert result.observability > 0.1


def test_corridor_is_unobservable():
    # Two parallel walls: translation along them is unconstrained
    xs = np.linspace(-5, 5, 400)
    corridor = np.vstack((np.column_stack((xs, np.full_like(xs, 1.0))),
                          np.column_stack((xs, np.full_like(xs, -1.0)))))
    result = icp(corridor, corridor)
    assert result.observability < 0.01


def test_transform_round_trip():
    points = np.array([[1.0, 0.0], [0.0, 2.0]])
    t = se2(0.5, -0.2, 0.7)
    back = transform_points(np.linalg.inv(t), transform_points(t, points))
    assert np.allclose(back, points)
