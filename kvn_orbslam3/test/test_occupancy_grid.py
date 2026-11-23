# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

import unittest

from kvn_orbslam3.occupancy_grid import GridConfig, OccupancyGrid2D, bresenham_cells


class TestOccupancyGrid(unittest.TestCase):
    def test_invalid_config_rejected(self):
        with self.assertRaises(ValueError):
            GridConfig(resolution=0.0).validate()
        with self.assertRaises(ValueError):
            GridConfig(min_range=5.0, max_range=2.0).validate()
        with self.assertRaises(ValueError):
            GridConfig(occ_threshold=0.2, free_threshold=0.5).validate()
        with self.assertRaises(ValueError):
            GridConfig(scan_stride=0).validate()

    def test_bresenham_line(self):
        cells = bresenham_cells(0, 0, 3, 2)
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (3, 2))
        self.assertEqual(len(cells), 4)

    def test_raytrace_marks_free_and_occupied(self):
        cfg = GridConfig(
            resolution=0.5,
            width_m=10.0,
            height_m=10.0,
            origin_x=-5.0,
            origin_y=-5.0,
            min_range=0.1,
            max_range=4.0,
        )
        grid = OccupancyGrid2D(cfg)
        # Single ray straight along +X hitting at 2.1 m from origin (0, 0)
        # Integrate 3 times so log-odds crosses both occ_threshold (0.65) and free_threshold (0.25)
        for _ in range(3):
            count = grid.integrate_scan(
                sensor_x=0.0,
                sensor_y=0.0,
                sensor_yaw=0.0,
                angle_min=0.0,
                angle_increment=0.1,
                ranges=[2.1],
            )
            self.assertEqual(count, 1)

        sx, sy = grid.world_to_grid(0.0, 0.0)
        hx, hy = grid.world_to_grid(2.1, 0.0)
        self.assertEqual(grid.cell_state(sx, sy), 0)
        self.assertEqual(grid.cell_state(hx, hy), 100)
        # Unobserved cell off the ray stays -1
        self.assertEqual(grid.cell_state(sx, sy + 3), -1)

    def test_max_range_clears_without_marking_occupied(self):
        cfg = GridConfig(
            resolution=0.5,
            width_m=10.0,
            height_m=10.0,
            origin_x=-5.0,
            origin_y=-5.0,
            max_range=3.0,
        )
        grid = OccupancyGrid2D(cfg)
        for _ in range(3):
            grid.integrate_scan(0.0, 0.0, 0.0, 0.0, 0.1, [10.0])
        ex, ey = grid.world_to_grid(3.0, 0.0)
        self.assertEqual(grid.cell_state(ex, ey), 0)

    def test_scan_stride_and_ros_export_length(self):
        cfg = GridConfig(
            resolution=1.0,
            width_m=6.0,
            height_m=4.0,
            origin_x=-3.0,
            origin_y=-2.0,
            scan_stride=2,
        )
        grid = OccupancyGrid2D(cfg)
        count = grid.integrate_scan(0.0, 0.0, 0.0, 0.0, 0.2, [1.5, 1.5, 1.5, 1.5])
        self.assertEqual(count, 2)
        data = grid.to_ros_data()
        self.assertEqual(len(data), 24)
        self.assertTrue(all(v == -1 or 0 <= v <= 100 for v in data))


if __name__ == '__main__':
    unittest.main()
