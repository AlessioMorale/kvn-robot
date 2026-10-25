# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""2D scan matching (point-to-line ICP) and SE(2) helpers. numpy only, no scipy."""
from dataclasses import dataclass
import math

import numpy as np


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_params(transform):
    return (float(transform[0, 2]), float(transform[1, 2]),
            math.atan2(transform[1, 0], transform[0, 0]))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_points(transform, points):
    return points @ transform[:2, :2].T + transform[:2, 2]


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan ranges -> (N, 2) points in the scan frame, invalid returns dropped."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.column_stack((r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])))


def voxel_downsample(points, voxel_size):
    """Replace the points in each voxel_size grid cell by their centroid."""
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points
    _, cell, counts = np.unique(
        np.floor(points / voxel_size).astype(np.int64), axis=0,
        return_inverse=True, return_counts=True)
    sums = np.zeros((counts.size, 2))
    np.add.at(sums, cell.ravel(), points)
    return sums / counts[:, None]


def _nearest(source, target, chunk=512):
    """Index and squared distance of the nearest target point for each source point."""
    index = np.empty(source.shape[0], dtype=np.int64)
    dist2 = np.empty(source.shape[0])
    for start in range(0, source.shape[0], chunk):
        d2 = ((source[start:start + chunk, None, :] - target[None, :, :]) ** 2).sum(axis=2)
        index[start:start + chunk] = np.argmin(d2, axis=1)
        dist2[start:start + chunk] = d2[np.arange(d2.shape[0]), index[start:start + chunk]]
    return index, dist2


def estimate_normals(points, neighbours=6):
    """Unit normal of each point from the PCA of its nearest neighbours."""
    k = min(neighbours, points.shape[0] - 1)
    d2 = ((points[:, None, :] - points[None, :, :]) ** 2).sum(axis=2)
    nbrs = points[np.argpartition(d2, k, axis=1)[:, :k + 1]]
    centred = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centred, centred)
    _, vectors = np.linalg.eigh(cov)
    return vectors[:, :, 0]  # eigenvector of the smallest eigenvalue


@dataclass
class IcpResult:
    transform: np.ndarray  # maps source points into the target frame
    rmse: float            # point-to-line RMS over inliers [m]
    inlier_ratio: float    # share of source points within inlier_distance of the target
    observability: float   # smallest eigenvalue of the translation information per point, 0..0.5
    iterations: int
    converged: bool


def icp(source, target, initial=None, max_iterations=60, max_correspondence=0.3,
        min_correspondence=0.05, inlier_distance=0.05, tolerance=1e-6):
    """Point-to-line ICP: the transform T with target ~= T * source.

    observability is ~0 when translation along some direction is unconstrained (e.g. a featureless
    corridor), in which case the translation along it is meaningless.
    """
    if source.shape[0] < 10 or target.shape[0] < 10:
        raise ValueError('ICP needs at least 10 points per scan')
    normals = estimate_normals(target)
    transform = np.eye(3) if initial is None else np.array(initial, dtype=float)
    converged = False
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        moved = transform_points(transform, source)
        index, dist2 = _nearest(moved, target)
        dist = np.sqrt(dist2)
        # Shrinking gate: generous while far off, tight once aligned
        gate = min(max_correspondence, max(min_correspondence, 3.0 * float(np.median(dist))))
        mask = dist < gate
        if np.count_nonzero(mask) < 10:
            break
        p, q, n = moved[mask], target[index[mask]], normals[index[mask]]
        # Residual (p - q).n, linearised in (tx, ty, dyaw) about the current pose
        jac = np.column_stack((n[:, 0], n[:, 1], p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]))
        res = ((p - q) * n).sum(axis=1)
        delta = -np.linalg.solve(jac.T @ jac + 1e-9 * np.eye(3), jac.T @ res)
        transform = se2(delta[0], delta[1], delta[2]) @ transform
        if np.hypot(delta[0], delta[1]) < tolerance and abs(delta[2]) < tolerance:
            converged = True
            break

    moved = transform_points(transform, source)
    index, dist2 = _nearest(moved, target)
    inliers = np.sqrt(dist2) < inlier_distance
    if np.count_nonzero(inliers) < 3:
        return IcpResult(transform, math.inf, 0.0, 0.0, iterations, False)
    n = normals[index[inliers]]
    res = ((moved[inliers] - target[index[inliers]]) * n).sum(axis=1)
    info = n.T @ n / n.shape[0]
    return IcpResult(
        transform=transform,
        rmse=float(np.sqrt(np.mean(res ** 2))),
        inlier_ratio=float(np.count_nonzero(inliers) / source.shape[0]),
        observability=float(np.linalg.eigvalsh(info)[0]),
        iterations=iterations,
        converged=converged)
