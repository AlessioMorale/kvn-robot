# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Extrinsics estimation: IMU floor compensation, lidar wall line, camera wall plane. numpy only.

Conventions: ROS REP 103/105 (x forward, y left, z up), rpy as in URDF joint origins (fixed axes
roll about x, then pitch about y, then yaw about z: R = Rz(yaw) Ry(pitch) Rx(roll)). A sensor's
rpy is the rotation of the sensor frame in base_link, so R maps sensor coordinates to base_link.
"""
from dataclasses import dataclass
import math

import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---- rotations -------------------------------------------------------------------------------

def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(r):
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def quaternion_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---- IMU floor compensation ------------------------------------------------------------------

def accel_roll_pitch(accel):
    """Roll and pitch of the sensor from the mean specific force of a stationary IMU, (N, 3).

    At rest the accelerometer reads +g along the up axis, so a sensor rolled by phi (pitched by
    theta) relative to level reads (0, g sin phi, g cos phi) ((-g sin theta, 0, g cos theta)).
    The mean vector is used rather than the mean of per-sample angles: it averages the noise
    before the non-linear step.
    """
    a = np.mean(np.atleast_2d(np.asarray(accel, dtype=float)), axis=0)
    if not np.all(np.isfinite(a)) or np.linalg.norm(a) < 1.0:
        raise ValueError(f'no gravity in the accelerometer data (mean {a.tolist()} m/s^2)')
    return math.atan2(a[1], a[2]), math.atan2(-a[0], math.hypot(a[1], a[2]))


@dataclass
class ImuBias:
    roll: float          # mounting roll of the IMU in base_link [rad]
    pitch: float         # mounting pitch [rad]
    floor_roll: float    # tilt of the floor under the rover at the first heading [rad]
    floor_pitch: float


def floor_compensated_bias(roll_pitch_0, roll_pitch_180):
    """IMU mounting roll/pitch from the tilt measured at two headings 180 deg apart.

    The floor's tilt seen in the rover frame changes sign after the half turn, the mounting
    tilt does not: the mean is the mounting, half the difference is the floor (small angles).
    """
    (r1, p1), (r2, p2) = roll_pitch_0, roll_pitch_180
    return ImuBias((r1 + r2) / 2.0, (p1 + p2) / 2.0, (r1 - r2) / 2.0, (p1 - p2) / 2.0)


# ---- RANSAC line / plane ---------------------------------------------------------------------

@dataclass
class HyperplaneFit:
    """n . p = distance with n the unit normal pointing away from the sensor (towards the wall)."""
    normal: np.ndarray
    distance: float      # [m] sensor origin to the line / plane
    inliers: int
    inlier_ratio: float
    rmse: float          # [m] point-to-line / plane over the inliers


def _orient(normal, offset):
    return (-normal, -offset) if offset < 0.0 else (normal, offset)


def _refine(points):
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    return _orient(normal, float(normal @ centroid))


def ransac_hyperplane(points, expected_normal=None, max_angle=math.radians(30.0), threshold=0.02,
                      iterations=2000, min_inliers=30, confidence=0.999, rng=None, batch=64):
    """Line (2D points) or plane (3D points) through the most points, refined by least squares.

    Only candidates whose normal is within max_angle of expected_normal (sign ignored) count, so
    that the floor or a side wall cannot win over the wall in front of the sensor. Candidates are
    scored in batches; the search stops once `confidence` that an all-inlier sample was drawn is
    reached for the best inlier ratio so far (a few dozen candidates for a wall filling the view),
    or after `iterations` candidates.
    """
    points = np.asarray(points, dtype=float)
    n_points, dim = points.shape if points.ndim == 2 else (0, 0)
    if n_points < max(min_inliers, dim):
        raise ValueError(f'only {n_points} points, need at least {max(min_inliers, dim)}')
    rng = np.random.default_rng() if rng is None else rng
    expected = None
    if expected_normal is not None:
        expected = np.asarray(expected_normal, dtype=float)
        expected = expected / np.linalg.norm(expected)
    min_cos = math.cos(max_angle)

    best_normal, best_offset, best_count = None, 0.0, 0
    drawn, needed = 0, iterations
    while drawn < min(needed, iterations):
        k = min(batch, iterations - drawn)
        drawn += k
        samples = points[rng.integers(0, n_points, (k, dim))]       # (k, dim, dim)
        diffs = samples[:, 1:] - samples[:, :1]
        if dim == 2:
            normals = np.column_stack((-diffs[:, 0, 1], diffs[:, 0, 0]))
        else:
            normals = np.cross(diffs[:, 0], diffs[:, 1])
        norms = np.linalg.norm(normals, axis=1)
        keep = norms > 1e-9                                         # repeated / collinear picks
        normals = normals[keep] / norms[keep, None]
        anchors = samples[keep, 0]
        if expected is not None:
            aligned = np.abs(normals @ expected) >= min_cos
            normals, anchors = normals[aligned], anchors[aligned]
        if normals.shape[0] == 0:
            continue
        offsets = np.einsum('kd,kd->k', normals, anchors)
        counts = np.count_nonzero(np.abs(points @ normals.T - offsets) < threshold, axis=0)
        i = int(np.argmax(counts))
        if counts[i] > best_count:
            best_normal, best_offset, best_count = normals[i], offsets[i], int(counts[i])
            w = best_count / n_points
            if w >= 1.0:
                break
            needed = math.log(1.0 - confidence) / math.log(1.0 - w ** dim)

    best_mask = None
    if best_normal is not None:
        best_mask = np.abs(points @ best_normal - best_offset) < threshold
    if best_count < min_inliers:
        raise ValueError(f'no {"line" if dim == 2 else "plane"} with at least {min_inliers} '
                         f'points within {math.degrees(max_angle):.0f} deg of the expected '
                         f'orientation (best: {best_count} of {n_points})')
    # Refinement: least squares on the inliers, then re-select them around the fit. The band
    # widens to 3 x the residual when the data is noisier than `threshold` assumes: a band that
    # truncates the noise keeps the fit tilted towards the random RANSAC candidate (measured: a
    # 2 cm band on 1.2 cm depth noise lost 0.7 deg of camera pitch).
    mask = best_mask
    for _ in range(5):
        normal, offset = _refine(points[mask])
        residual = points[mask] @ normal - offset
        band = max(threshold, 3.0 * float(np.sqrt(np.mean(residual ** 2))))
        new_mask = np.abs(points @ normal - offset) < band
        if np.count_nonzero(new_mask) < dim + 1:
            break
        converged = np.array_equal(new_mask, mask)
        mask = new_mask
        if converged:
            break
    normal, offset = _refine(points[mask])
    residual = points[mask] @ normal - offset
    return HyperplaneFit(normal, offset, int(np.count_nonzero(mask)),
                         float(np.count_nonzero(mask)) / n_points,
                         float(np.sqrt(np.mean(residual ** 2))))


# ---- LiDAR yaw -------------------------------------------------------------------------------

def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   window_center=0.0, window_half_width=math.pi):
    """LaserScan -> (N, 2) points in the scan frame, within +-window_half_width of window_center."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    offset = np.arctan2(np.sin(a - window_center), np.cos(a - window_center))
    valid = (np.isfinite(r) & (r > range_min) & (r < range_max) &
             (np.abs(offset) <= window_half_width))
    return np.column_stack((r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])))


def wall_normal_angle(fit):
    """Direction of the wall normal (sensor -> wall) in the sensor's xy plane [rad]."""
    return math.atan2(fit.normal[1], fit.normal[0])


def lidar_yaw_from_wall(fit, wall_normal_in_base=0.0):
    """Yaw of the lidar frame in base_link from a wall line fitted in the lidar frame.

    With the rover square to the wall, the wall normal lies along base_link +x
    (wall_normal_in_base = 0). A lidar yawed by psi sees that direction at -psi, so
    psi = wall_normal_in_base - (normal angle in the lidar frame). For a line Ax + By + C = 0 this
    is atan2(-A, B) - pi/2 up to the sign of the normal, which is fixed here to point at the wall.
    """
    return wrap_angle(wall_normal_in_base - wall_normal_angle(fit))


# ---- Camera pitch / yaw ----------------------------------------------------------------------

def camera_pitch_yaw(normal_camera, normal_base):
    """Pitch and yaw of the camera frame in base_link (roll taken as 0) from one wall normal.

    normal_camera: wall normal seen in the camera mount frame (x forward). normal_base: the same
    normal in base_link, horizontal (from the lidar). Solves Rz(yaw) Ry(pitch) n_c = n_b: the
    pitch levels n_c, the yaw then turns it onto n_b. The roll about the normal is not observable
    from a single plane.
    """
    n = np.asarray(normal_camera, dtype=float)
    n = n / np.linalg.norm(n)
    pitch = math.atan2(n[2], n[0])
    levelled_x = math.cos(pitch) * n[0] + math.sin(pitch) * n[2]
    yaw = wrap_angle(math.atan2(normal_base[1], normal_base[0]) - math.atan2(n[1], levelled_x))
    return pitch, yaw


def remove_plane(points, normal_hint, max_angle=math.radians(20.0), threshold=0.02,
                 min_inliers=200, rng=None):
    """Points with the dominant plane near normal_hint (e.g. the floor) taken out.

    The removal band is twice the fit threshold wide, so that the strip of floor along the base of
    a wall goes too: left in, it would tilt the wall's fit. Without such a plane the points come
    back unchanged.
    """
    try:
        fit = ransac_hyperplane(points, normal_hint, max_angle, threshold,
                                min_inliers=min_inliers, rng=rng)
    except ValueError:
        return points
    return points[np.abs(points @ fit.normal - fit.distance) >= 2.0 * threshold]


def crop_forward(points, min_range, max_range, half_width_angle, half_height_angle):
    """Points (N, 3) in an x-forward frame inside a pyramid around the optical axis."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    keep = ((x > min_range) & (x < max_range) &
            (np.abs(y) < x * math.tan(half_width_angle)) &
            (np.abs(z) < x * math.tan(half_height_angle)))
    return points[keep]


def subsample(points, max_points, rng=None):
    if points.shape[0] <= max_points:
        return points
    rng = np.random.default_rng() if rng is None else rng
    return points[rng.choice(points.shape[0], max_points, replace=False)]
