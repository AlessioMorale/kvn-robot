import math

import pytest

from kvn_odom_calibration.fitting import (
    SPIN, STRAIGHT, Segment, fit_chi, fit_gyro_scale, fit_wheel_radius, straight_heading_drift)

RADIUS, TRACK, CHI = 0.037, 0.125, 1.6


def straight(distance, yaw=0.0):
    phi = distance / RADIUS
    return Segment(STRAIGHT, 1 if distance > 0 else -1, phi, phi, distance, 0.0, yaw)


def spin(yaw, gyro_scale=1.0):
    dphi = yaw * TRACK * CHI / RADIUS
    return Segment(SPIN, 1 if yaw > 0 else -1, -dphi / 2, dphi / 2, 0.0, 0.0, yaw,
                   gyro_yaw=gyro_scale * yaw)


def test_fits_recover_true_parameters():
    segments = [straight(1.0), straight(-1.0), spin(2 * math.pi, 1.01), spin(-2 * math.pi, 1.01)]
    radius = fit_wheel_radius(segments)
    assert radius.value == pytest.approx(RADIUS)
    assert fit_chi(segments, radius.value, TRACK).value == pytest.approx(CHI)
    assert fit_gyro_scale(segments) == pytest.approx(1.01)


def test_invalid_segments_are_ignored():
    bad = straight(1.0)
    bad.ref_x = 5.0
    bad.valid = False
    assert fit_wheel_radius([straight(1.0), bad]).value == pytest.approx(RADIUS)
    assert fit_chi([straight(1.0)], RADIUS, TRACK) is None


def test_heading_drift_sign_is_independent_of_direction():
    drift = straight_heading_drift([straight(1.0, yaw=0.02), straight(-1.0, yaw=-0.02)])
    assert drift == pytest.approx(0.02)
