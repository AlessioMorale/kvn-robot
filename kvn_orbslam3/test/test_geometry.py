# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

import math
import unittest

from kvn_orbslam3.geometry import (
    Quaternion,
    Transform3D,
    compute_map_to_base,
    compute_map_to_odom,
)


class TestGeometry(unittest.TestCase):
    def assert_tuple_almost(self, actual, expected, places=6):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=places)

    def test_quaternion_identity_and_yaw(self):
        q = Quaternion.from_yaw(math.pi / 2.0)
        self.assertAlmostEqual(q.yaw(), math.pi / 2.0, places=6)
        rx, ry, rz = q.rotate_vector((1.0, 0.0, 0.0))
        self.assert_tuple_almost((rx, ry, rz), (0.0, 1.0, 0.0))

    def test_transform_inverse_round_trip(self):
        tf = Transform3D(
            (1.2, -0.4, 0.35),
            Quaternion.from_rpy(0.15, -0.2, 0.8),
        )
        ident = tf.compose(tf.inverse())
        self.assert_tuple_almost(ident.translation, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(abs(ident.rotation.w), 1.0, places=6)

    def test_compute_map_to_base_with_optical_frame(self):
        # Camera sits at (+0.074, 0, +0.10) with optical frame rpy=(-pi/2, 0, -pi/2)
        t_base_sensor = Transform3D(
            (0.074, 0.0, 0.10),
            Quaternion.from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
        )
        # Robot at origin in map -> sensor pose in map equals t_base_sensor
        t_map_base = compute_map_to_base(t_base_sensor, t_base_sensor)
        self.assert_tuple_almost(t_map_base.translation, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(abs(t_map_base.rotation.w), 1.0, places=6)

    def test_compute_map_to_odom_3d_round_trip(self):
        t_base_sensor = Transform3D(
            (0.074, 0.0, 0.10),
            Quaternion.from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
        )
        t_map_base_true = Transform3D(
            (2.0, 1.0, 0.0),
            Quaternion.from_yaw(0.5),
        )
        t_odom_base = Transform3D(
            (1.5, 0.2, 0.0),
            Quaternion.from_yaw(0.25),
        )
        t_map_sensor = t_map_base_true.compose(t_base_sensor)

        t_map_odom, t_map_base_3d = compute_map_to_odom(
            t_map_sensor, t_base_sensor, t_odom_base, project_to_2d=False
        )
        reconstructed = t_map_odom.compose(t_odom_base)
        self.assert_tuple_almost(reconstructed.translation, t_map_base_true.translation)
        self.assertAlmostEqual(reconstructed.rotation.yaw(), t_map_base_true.rotation.yaw(), places=6)
        self.assert_tuple_almost(t_map_base_3d.translation, t_map_base_true.translation)

    def test_compute_map_to_odom_flattens_2d(self):
        t_base_sensor = Transform3D((0.074, 0.0, 0.10), Quaternion.identity())
        # Inject VIO z-drift, roll and pitch
        t_map_base_noisy = Transform3D(
            (1.0, -0.5, 0.18),
            Quaternion.from_rpy(0.12, -0.09, 0.7),
        )
        t_odom_base = Transform3D((0.8, -0.4, 0.0), Quaternion.from_yaw(0.6))
        t_map_sensor = t_map_base_noisy.compose(t_base_sensor)

        t_map_odom, t_map_base_3d = compute_map_to_odom(
            t_map_sensor, t_base_sensor, t_odom_base, project_to_2d=True
        )
        self.assertAlmostEqual(t_map_odom.translation[2], 0.0, places=6)
        self.assertAlmostEqual(t_map_odom.rotation.x, 0.0, places=6)
        self.assertAlmostEqual(t_map_odom.rotation.y, 0.0, places=6)
        reconstructed = t_map_odom.compose(t_odom_base)
        self.assert_tuple_almost(reconstructed.translation, (1.0, -0.5, 0.0))
        self.assertAlmostEqual(reconstructed.rotation.yaw(), 0.7, places=6)
        # 3D pose is preserved separately
        self.assertAlmostEqual(t_map_base_3d.translation[2], 0.18, places=6)


if __name__ == '__main__':
    unittest.main()
