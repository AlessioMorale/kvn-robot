# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Parser and Pi 5 envelope validator for ORB-SLAM3 OpenCV YAML settings files.

Enforces the Raspberry Pi 5 (Cortex-A76) constraints documented in orbslam3.md:
  - Resolution hard-capped at 640x480 (VGA)
  - Camera framerate <= 30 Hz (15 Hz recommended)
  - IMU frequency in [200, 400] Hz (for hardware profile)
  - ORBextractor.nFeatures in [600, 800]
  - ORBextractor.nLevels in [4, 6]
  - Headless operation (Viewer.UseViewer == 0)
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any


@dataclass(frozen=True)
class Pi5Envelope:
    max_width: int = 640
    max_height: int = 480
    max_fps: float = 30.0
    min_imu_hz: float = 200.0
    max_imu_hz: float = 400.0
    min_features: int = 600
    max_features: int = 800
    min_levels: int = 4
    max_levels: int = 6
    require_headless: bool = True


_KV_RE = re.compile(r'^([A-Za-z0-9_.\-]+)\s*:\s*(.*)$')


def _parse_scalar(raw: str) -> Any:
    val = raw.strip()
    if not val:
        return ''
    if (val.startswith('"') and val.endswith('"')) or (
        val.startswith("'") and val.endswith("'")
    ):
        return val[1:-1]
    try:
        return int(val)
    except ValueError:
        pass
    try:
        return float(val)
    except ValueError:
        return val


def parse_orbslam3_yaml(text: str) -> dict[str, Any]:
    """Parse top-level scalar keys from an OpenCV/ORB-SLAM3 YAML 1.0 file.

    Avoids requiring PyYAML for OpenCV-specific tags like `%YAML:1.0` and
    `!!opencv-matrix`, which standard yaml.safe_load rejects.
    """
    result: dict[str, Any] = {}
    for raw_line in text.splitlines():
        line = raw_line.split('#', 1)[0].rstrip()
        if not line or line.startswith('%') or line.startswith('---'):
            continue
        if line[0].isspace():
            continue
        match = _KV_RE.match(line)
        if not match:
            continue
        key, raw_val = match.group(1), match.group(2).strip()
        if raw_val.startswith('!!'):
            result[key] = raw_val
            continue
        result[key] = _parse_scalar(raw_val)
    return result


def validate_pi5_settings(
    settings: dict[str, Any],
    envelope: Pi5Envelope | None = None,
    check_imu_rate: bool = True,
) -> list[str]:
    """Return a list of constraint violations (empty list means valid)."""
    env = envelope or Pi5Envelope()
    errors: list[str] = []

    def require_num(key: str) -> float | None:
        val = settings.get(key)
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            errors.append(f"missing or non-numeric '{key}'")
            return None
        return float(val)

    width = require_num('Camera.width')
    if width is not None and not (0 < width <= env.max_width):
        errors.append(f'Camera.width={width} exceeds Pi 5 VGA cap ({env.max_width})')

    height = require_num('Camera.height')
    if height is not None and not (0 < height <= env.max_height):
        errors.append(f'Camera.height={height} exceeds Pi 5 VGA cap ({env.max_height})')

    fps = require_num('Camera.fps')
    if fps is not None and not (0 < fps <= env.max_fps):
        errors.append(f'Camera.fps={fps} exceeds Pi 5 cap ({env.max_fps} Hz)')

    if check_imu_rate:
        imu_hz = require_num('IMU.Frequency')
        if imu_hz is not None and not (env.min_imu_hz <= imu_hz <= env.max_imu_hz):
            errors.append(
                f'IMU.Frequency={imu_hz} outside [{env.min_imu_hz}, {env.max_imu_hz}] Hz'
            )

    n_features = require_num('ORBextractor.nFeatures')
    if n_features is not None and not (env.min_features <= n_features <= env.max_features):
        errors.append(
            f'ORBextractor.nFeatures={n_features} outside [{env.min_features}, {env.max_features}]'
        )

    n_levels = require_num('ORBextractor.nLevels')
    if n_levels is not None and not (env.min_levels <= n_levels <= env.max_levels):
        errors.append(
            f'ORBextractor.nLevels={n_levels} outside [{env.min_levels}, {env.max_levels}]'
        )

    if env.require_headless:
        viewer = settings.get('Viewer.UseViewer', 0)
        if viewer not in (0, False, 'false'):
            errors.append(f'Viewer.UseViewer={viewer} must be 0 (headless on Pi 5)')

    if 'IMU.T_b_c1' not in settings:
        errors.append("missing 'IMU.T_b_c1' camera-to-IMU extrinsic matrix")

    return errors
