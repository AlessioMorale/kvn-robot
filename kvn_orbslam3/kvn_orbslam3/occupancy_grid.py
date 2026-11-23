# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Lightweight bounded 2D log-odds occupancy grid for ORB-SLAM3 + 2D LiDAR.

ORB-SLAM3 estimates 3D pose (map -> odom via orbslam3_tf_bridge) but does not publish
a 2D nav_msgs/OccupancyGrid on /map. Nav2's static_layer, explore_lite and
map_saver_server all expect /map. This module maintains a fixed-bound log-odds grid
so memory usage stays strictly capped on a Raspberry Pi 5 (4 GB or 8 GB).
"""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class GridConfig:
    """Parameters for OccupancyGrid2D."""

    resolution: float = 0.05
    width_m: float = 20.0
    height_m: float = 20.0
    origin_x: float = -10.0
    origin_y: float = -10.0
    min_range: float = 0.15
    max_range: float = 8.0
    l_occ: float = 0.85
    l_free: float = -0.40
    l_min: float = -2.0
    l_max: float = 3.5
    occ_threshold: float = 0.65
    free_threshold: float = 0.25
    scan_stride: int = 1

    def validate(self) -> None:
        if self.resolution <= 0.0 or not math.isfinite(self.resolution):
            raise ValueError(f'resolution must be > 0, got {self.resolution}')
        if self.width_m <= 0.0 or self.height_m <= 0.0:
            raise ValueError('width_m and height_m must be > 0')
        if self.min_range < 0.0 or self.max_range <= self.min_range:
            raise ValueError('must satisfy 0 <= min_range < max_range')
        if self.l_occ <= 0.0 or self.l_free >= 0.0:
            raise ValueError('must satisfy l_occ > 0 and l_free < 0')
        if self.l_min >= 0.0 or self.l_max <= 0.0:
            raise ValueError('must satisfy l_min < 0 < l_max')
        if not (0.0 < self.free_threshold < self.occ_threshold < 1.0):
            raise ValueError('must satisfy 0 < free_threshold < occ_threshold < 1')
        if self.scan_stride < 1:
            raise ValueError('scan_stride must be >= 1')


def bresenham_cells(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Return grid cells along the segment from (x0, y0) to (x1, y1), inclusive."""
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class OccupancyGrid2D:
    """Fixed-size 2D log-odds occupancy grid with bounded RAM footprint."""

    def __init__(self, config: GridConfig | None = None) -> None:
        self.config = config or GridConfig()
        self.config.validate()
        self.width = int(round(self.config.width_m / self.config.resolution))
        self.height = int(round(self.config.height_m / self.config.resolution))
        self.origin_x = self.config.origin_x
        self.origin_y = self.config.origin_y
        size = self.width * self.height
        # float32 log-odds + uint8 observed mask: ~5 bytes/cell (400x400 = ~800 KB)
        self._log_odds = array('f', [0.0]) * size
        self._observed = bytearray(size)
        self._l_occ_thresh = math.log(
            self.config.occ_threshold / (1.0 - self.config.occ_threshold)
        )
        self._l_free_thresh = math.log(
            self.config.free_threshold / (1.0 - self.config.free_threshold)
        )

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        gx = int(math.floor((wx - self.origin_x) / self.config.resolution))
        gy = int(math.floor((wy - self.origin_y) / self.config.resolution))
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def _update_cell(self, gx: int, gy: int, delta: float) -> None:
        if not self.in_bounds(gx, gy):
            return
        idx = self._index(gx, gy)
        val = self._log_odds[idx] + delta
        if val > self.config.l_max:
            val = self.config.l_max
        elif val < self.config.l_min:
            val = self.config.l_min
        self._log_odds[idx] = val
        self._observed[idx] = 1

    def integrate_scan(
        self,
        sensor_x: float,
        sensor_y: float,
        sensor_yaw: float,
        angle_min: float,
        angle_increment: float,
        ranges: Iterable[float],
    ) -> int:
        """Integrate one planar LaserScan posed in map frame.

        Returns the number of rays integrated.
        """
        sx, sy = self.world_to_grid(sensor_x, sensor_y)
        if not self.in_bounds(sx, sy):
            return 0

        cfg = self.config
        integrated = 0
        for i, r in enumerate(ranges):
            if i % cfg.scan_stride != 0:
                continue
            if not math.isfinite(r) or r < cfg.min_range:
                continue

            is_hit = r <= cfg.max_range
            ray_len = r if is_hit else cfg.max_range
            angle = sensor_yaw + angle_min + i * angle_increment
            ex, ey = self.world_to_grid(
                sensor_x + ray_len * math.cos(angle),
                sensor_y + ray_len * math.sin(angle),
            )
            cells = bresenham_cells(sx, sy, ex, ey)
            # Mark all cells strictly before the endpoint as free
            for gx, gy in cells[:-1]:
                self._update_cell(gx, gy, cfg.l_free)
            if is_hit:
                self._update_cell(ex, ey, cfg.l_occ)
            else:
                self._update_cell(ex, ey, cfg.l_free)
            integrated += 1
        return integrated

    def to_ros_data(self) -> list[int]:
        """Export ROS nav_msgs/OccupancyGrid data (-1 unknown, 0 free, 100 occupied)."""
        out = [-1] * (self.width * self.height)
        l_occ = self._l_occ_thresh
        l_free = self._l_free_thresh
        log_odds = self._log_odds
        observed = self._observed
        for i, seen in enumerate(observed):
            if not seen:
                continue
            lo = log_odds[i]
            if lo >= l_occ:
                out[i] = 100
            elif lo <= l_free:
                out[i] = 0
            else:
                # Uncertain observed cell: map probability smoothly to [1, 99]
                p = 1.0 / (1.0 + math.exp(-lo))
                out[i] = max(1, min(99, int(round(100.0 * p))))
        return out

    def cell_state(self, gx: int, gy: int) -> int:
        """Return ROS occupancy value (-1, 0..100) for a single cell."""
        if not self.in_bounds(gx, gy):
            return -1
        idx = self._index(gx, gy)
        if not self._observed[idx]:
            return -1
        lo = self._log_odds[idx]
        if lo >= self._l_occ_thresh:
            return 100
        if lo <= self._l_free_thresh:
            return 0
        p = 1.0 / (1.0 + math.exp(-lo))
        return max(1, min(99, int(round(100.0 * p))))
