import math

import numpy as np
import pytest

from kvn_calibration_supervisor import algorithms as alg


def test_rpy_matrix_round_trip():
    for rpy in [(0.0, 0.0, 0.0), (0.1, -0.2, 0.3), (0.0, 0.05, -math.pi / 2)]:
        assert alg.matrix_to_rpy(alg.rpy_to_matrix(*rpy)) == pytest.approx(rpy, abs=1e-12)


def test_quaternion_matches_rpy():
    roll, pitch, yaw = 0.1, -0.2, 0.3
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    assert np.allclose(alg.quaternion_to_matrix(*q), alg.rpy_to_matrix(roll, pitch, yaw))


def _accel(rotation_sensor_in_world, rng, n=500):
    """Specific force of a stationary IMU whose frame has the given rotation in the world."""
    f = rotation_sensor_in_world.T @ np.array([0.0, 0.0, 9.81])
    return f + rng.normal(0.0, 0.02, (n, 3))


def test_accel_roll_pitch():
    rng = np.random.default_rng(0)
    roll, pitch = alg.accel_roll_pitch(_accel(alg.rpy_to_matrix(0.03, -0.02, 0.7), rng))
    assert roll == pytest.approx(0.03, abs=1e-3)
    assert pitch == pytest.approx(-0.02, abs=1e-3)


def test_accel_without_gravity_is_rejected():
    with pytest.raises(ValueError):
        alg.accel_roll_pitch(np.zeros((10, 3)))


def test_floor_compensation_cancels_the_floor_tilt():
    rng = np.random.default_rng(1)
    mount = alg.rpy_to_matrix(0.02, -0.015, 0.0)        # IMU in base_link
    floor = alg.rpy_to_matrix(0.03, 0.04, 0.0)          # floor tilt under the rover
    measured = []
    for heading in (0.0, math.pi):
        base = floor @ alg.rpy_to_matrix(0.0, 0.0, heading)
        measured.append(alg.accel_roll_pitch(_accel(base @ mount, rng)))
    bias = alg.floor_compensated_bias(*measured)
    assert bias.roll == pytest.approx(0.02, abs=1e-3)
    assert bias.pitch == pytest.approx(-0.015, abs=1e-3)
    # The uncompensated first reading is dominated by the floor
    assert abs(measured[0][0] - 0.02) > 0.02


def _scan_of_wall(lidar_yaw, wall_x=1.0, lidar_x=0.03, n=720, noise=0.003, rng=None):
    """Ranges of a 360 deg scan seeing a wall at base x = wall_x and a side wall at y = 0.6."""
    rng = rng or np.random.default_rng(2)
    angles = -math.pi + 2 * math.pi * np.arange(n) / n
    ranges = np.full(n, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a + lidar_yaw), math.sin(a + lidar_yaw)])  # ray in base
        hits = []
        if d[0] > 1e-6:
            hits.append((wall_x - lidar_x) / d[0])
        if d[1] > 1e-6:
            hits.append(0.6 / d[1])
        if hits:
            ranges[i] = min(hits) + rng.normal(0.0, noise)
    return ranges, -math.pi, 2 * math.pi / n


@pytest.mark.parametrize('yaw_offset', [0.0, 0.03, -0.05])
def test_lidar_yaw_from_wall(yaw_offset):
    lidar_yaw = -math.pi / 2 + yaw_offset  # URDF mounting plus the error to find
    ranges, angle_min, inc = _scan_of_wall(lidar_yaw)
    center = math.pi / 2  # base +x seen from the nominal (URDF) lidar yaw
    points = alg.scan_to_points(ranges, angle_min, inc, 0.05, 3.0, center, 0.5)
    fit = alg.ransac_hyperplane(points, (math.cos(center), math.sin(center)),
                                threshold=0.015, rng=np.random.default_rng(3))
    assert alg.lidar_yaw_from_wall(fit) == pytest.approx(lidar_yaw, abs=math.radians(0.2))
    assert fit.distance == pytest.approx(0.97, abs=0.01)


def test_ransac_ignores_planes_off_the_expected_orientation():
    rng = np.random.default_rng(4)
    floor = np.column_stack((rng.uniform(0.5, 2, 2000), rng.uniform(-1, 1, 2000),
                             np.full(2000, -0.1)))
    wall = np.column_stack((np.full(300, 1.5), rng.uniform(-0.5, 0.5, 300),
                            rng.uniform(-0.1, 0.5, 300)))
    fit = alg.ransac_hyperplane(np.vstack((floor, wall)), (1.0, 0.0, 0.0), rng=rng)
    # The strip of floor at the base of the wall biases it slightly (<0.2 deg)
    assert fit.normal == pytest.approx([1.0, 0.0, 0.0], abs=3e-3)
    assert fit.distance == pytest.approx(1.5, abs=1e-3)
    with pytest.raises(ValueError):
        alg.ransac_hyperplane(floor, (1.0, 0.0, 0.0), rng=rng)


@pytest.mark.parametrize('pitch,yaw', [(0.0, 0.0), (0.2, 0.04), (-0.05, -0.03)])
def test_camera_pitch_yaw_from_wall_plane(pitch, yaw):
    rng = np.random.default_rng(5)
    r_cam = alg.rpy_to_matrix(0.0, pitch, yaw)          # camera mount in base_link
    t_cam = np.array([0.074, 0.0, 0.10])
    # Wall at base x = 1.2 seen by the camera, plus the floor at z = 0
    wall = np.column_stack((np.full(4000, 1.2), rng.uniform(-0.6, 0.6, 4000),
                            rng.uniform(0.0, 0.6, 4000)))
    floor = np.column_stack((rng.uniform(0.3, 1.2, 4000), rng.uniform(-0.6, 0.6, 4000),
                             np.zeros(4000)))
    points_cam = (np.vstack((wall, floor)) - t_cam) @ r_cam  # base -> camera: R^T (p - t)
    points_cam += rng.normal(0.0, 0.004, points_cam.shape)
    points_cam = alg.crop_forward(points_cam, 0.3, 2.5, 0.35, 0.25)
    nominal = alg.rpy_to_matrix(0.0, 0.0, 0.0)  # URDF mount, without the error to find
    points_cam = alg.remove_plane(points_cam, nominal.T @ [0.0, 0.0, 1.0], rng=rng)
    normal_base = np.array([1.0, 0.0, 0.0])
    expected = nominal.T @ normal_base
    plane = alg.ransac_hyperplane(points_cam, expected, threshold=0.015, rng=rng)
    got_pitch, got_yaw = alg.camera_pitch_yaw(plane.normal, normal_base)
    assert got_pitch == pytest.approx(pitch, abs=math.radians(0.3))
    assert got_yaw == pytest.approx(yaw, abs=math.radians(0.3))


def test_camera_yaw_follows_the_lidar_wall_normal():
    # Rover 2 deg off square to the wall: the normal is no longer base +x
    wall = 0.035
    n_base = np.array([math.cos(wall), math.sin(wall), 0.0])
    n_cam = alg.rpy_to_matrix(0.0, 0.1, 0.02).T @ n_base
    assert alg.camera_pitch_yaw(n_cam, n_base) == pytest.approx((0.1, 0.02), abs=1e-9)


def test_remove_plane_keeps_points_without_that_plane():
    rng = np.random.default_rng(6)
    wall = np.column_stack((np.full(500, 1.0), rng.uniform(-1, 1, 500), rng.uniform(0, 1, 500)))
    assert alg.remove_plane(wall, (0.0, 0.0, 1.0), rng=rng).shape == wall.shape


def test_noisy_wall_is_not_biased_by_a_tight_threshold():
    # Depth noise (1.2 cm) larger than the inlier threshold must not tilt the fit
    rng = np.random.default_rng(7)
    pitch = 0.05
    wall = np.column_stack((np.full(20000, 1.0), rng.uniform(-0.3, 0.3, 20000),
                            rng.uniform(-0.2, 0.2, 20000)))
    points = wall @ alg.rpy_to_matrix(0.0, pitch, 0.0)
    points[:, 0] += rng.normal(0.0, 0.012, points.shape[0])
    for threshold in (0.01, 0.02):
        fit = alg.ransac_hyperplane(points, (1.0, 0.0, 0.0), threshold=threshold, rng=rng)
        got, _ = alg.camera_pitch_yaw(fit.normal, (1.0, 0.0, 0.0))
        assert got == pytest.approx(pitch, abs=math.radians(0.15))
