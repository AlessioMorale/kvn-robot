# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Least-squares fit of wheel_radius and chi from calibration segments.

Same kinematic model as kvn_odometry and skid_steer_controller, with phi_left / phi_right the wheel
rotation [rad] of each side (front/back averaged) over one segment:
  distance = wheel_radius * (phi_left + phi_right) / 2
  yaw      = wheel_radius * (phi_right - phi_left) / (track_width * chi)
"""
from dataclasses import dataclass, field
import math

STRAIGHT = 'straight'
SPIN = 'spin'


@dataclass
class Segment:
    kind: str                # STRAIGHT or SPIN
    direction: int           # +1 forward / counter-clockwise, -1 backward / clockwise
    phi_left: float          # [rad]
    phi_right: float         # [rad]
    ref_x: float             # rover motion from the reference (lidar), start base frame [m]
    ref_y: float
    ref_yaw: float           # unwrapped [rad]
    gyro_yaw: float = math.nan  # bias-corrected integrated gyro [rad]
    rmse: float = math.nan
    inlier_ratio: float = math.nan
    observability: float = math.nan
    valid: bool = True
    note: str = ''

    @property
    def ref_distance(self):
        # Signed: odometry only models forward motion, so drive direction gives the sign
        return math.copysign(math.hypot(self.ref_x, self.ref_y), self.ref_x)


@dataclass
class Estimate:
    value: float
    spread: float            # std of the per-segment values
    samples: list = field(default_factory=list)


def _std(values):
    if len(values) < 2:
        return math.nan
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


def fit_wheel_radius(segments):
    """wheel_radius minimising the distance error over the straight segments."""
    data = [((s.phi_left + s.phi_right) / 2.0, s.ref_distance)
            for s in segments if s.kind == STRAIGHT and s.valid]
    data = [(phi, d) for phi, d in data if abs(phi) > 1e-6]
    if not data:
        return None
    radius = sum(phi * d for phi, d in data) / sum(phi * phi for phi, _ in data)
    samples = [d / phi for phi, d in data]
    return Estimate(radius, _std(samples), samples)


def fit_chi(segments, wheel_radius, track_width):
    """chi minimising the yaw error over the spin segments, for the given wheel_radius."""
    data = [(s.phi_right - s.phi_left, s.ref_yaw)
            for s in segments if s.kind == SPIN and s.valid]
    data = [(dphi, yaw) for dphi, yaw in data if abs(yaw) > 1e-3]
    if not data:
        return None
    # yaw = k * dphi with k = wheel_radius / (track_width * chi)
    k = sum(dphi * yaw for dphi, yaw in data) / sum(dphi * dphi for dphi, _ in data)
    if k <= 0.0:
        return None
    samples = [wheel_radius * dphi / (track_width * yaw) for dphi, yaw in data]
    return Estimate(wheel_radius / (track_width * k), _std(samples), samples)


def fit_gyro_scale(segments):
    """Integrated gyro yaw / reference yaw over the spins; 1.0 for a well-scaled IMU."""
    data = [(s.gyro_yaw, s.ref_yaw) for s in segments
            if s.kind == SPIN and s.valid and math.isfinite(s.gyro_yaw)]
    if not data:
        return None
    return sum(g * y for g, y in data) / sum(y * y for _, y in data)


def straight_heading_drift(segments):
    """Mean yaw per metre driven on the straight segments [rad/m] (left/right imbalance)."""
    data = [s.ref_yaw / s.ref_distance for s in segments
            if s.kind == STRAIGHT and s.valid and abs(s.ref_distance) > 0.05]
    return sum(data) / len(data) if data else None
