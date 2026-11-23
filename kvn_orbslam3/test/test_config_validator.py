# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

from pathlib import Path
import unittest

from kvn_orbslam3.config_validator import parse_orbslam3_yaml, validate_pi5_settings

CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'


class TestConfigValidator(unittest.TestCase):
    def test_shipped_pi5_config_passes_envelope(self):
        text = (CONFIG_DIR / 'oak_d_lite_rgbd_inertial_pi5.yaml').read_text(encoding='utf-8')
        parsed = parse_orbslam3_yaml(text)
        errors = validate_pi5_settings(parsed, check_imu_rate=True)
        self.assertEqual(errors, [], f'Unexpected Pi 5 validation errors: {errors}')
        self.assertEqual(parsed['Camera.width'], 640)
        self.assertEqual(parsed['Camera.height'], 400)
        self.assertEqual(parsed['Camera.fps'], 15)
        self.assertEqual(parsed['Viewer.UseViewer'], 0)

    def test_shipped_sim_config_passes_envelope(self):
        text = (CONFIG_DIR / 'sim_rgbd_inertial.yaml').read_text(encoding='utf-8')
        parsed = parse_orbslam3_yaml(text)
        errors = validate_pi5_settings(parsed, check_imu_rate=False)
        self.assertEqual(errors, [], f'Unexpected sim validation errors: {errors}')

    def test_rejects_oversized_resolution_fps_and_features(self):
        bad = {
            'Camera.width': 1280,
            'Camera.height': 720,
            'Camera.fps': 60,
            'IMU.Frequency': 50.0,
            'ORBextractor.nFeatures': 1200,
            'ORBextractor.nLevels': 8,
            'Viewer.UseViewer': 1,
        }
        errors = validate_pi5_settings(bad, check_imu_rate=True)
        joined = '\n'.join(errors)
        self.assertIn('Camera.width', joined)
        self.assertIn('Camera.height', joined)
        self.assertIn('Camera.fps', joined)
        self.assertIn('IMU.Frequency', joined)
        self.assertIn('ORBextractor.nFeatures', joined)
        self.assertIn('ORBextractor.nLevels', joined)
        self.assertIn('Viewer.UseViewer', joined)
        self.assertIn('IMU.T_b_c1', joined)


if __name__ == '__main__':
    unittest.main()
