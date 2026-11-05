import yaml

from kvn_calibration_supervisor import persistence


def _sensors(lidar_yaw=-1.5708):
    return {
        'imu': {'translation': [-0.05, -0.03, 0.025], 'rotation_rpy': [0.01, -0.02, 0.0]},
        'oakd': {'translation': [0.074, 0.0, 0.1], 'rotation_rpy': [0.0, 0.1, 0.01]},
        'lidar': {'translation': [0.03, 0.0, 0.115], 'rotation_rpy': [0.0, 0.0, lidar_yaw]},
    }


def test_layout_matches_ros_parameters(tmp_path):
    path = tmp_path / 'calibration_test.yaml'
    odom = {'wheel_radius': 0.0362, 'track_width': 0.125, 'chi': 1.31}
    persistence.save(str(path), persistence.build_update(
        _sensors(), odom, ('/skid_steer_controller', '/odometry_node')))
    text = path.read_text()
    assert text.startswith('# KVN calibration')
    data = yaml.safe_load(text)
    assert list(data) == ['/**', 'odometry_node', 'skid_steer_controller']
    ext = data['/**']['ros__parameters']['extrinsics']
    assert set(ext) == {'base_link_to_imu', 'base_link_to_oakd', 'base_link_to_lidar'}
    assert ext['base_link_to_lidar']['rotation_rpy'] == [0.0, 0.0, -1.5708]
    assert data['skid_steer_controller']['ros__parameters'] == odom
    assert persistence.extrinsics(data)['imu']['rotation_rpy'] == [0.01, -0.02, 0.0]


def test_rerun_overwrites_without_duplicates_and_keeps_skipped(tmp_path):
    path = str(tmp_path / 'calibration_test.yaml')
    odom = {'wheel_radius': 0.0362, 'track_width': 0.125, 'chi': 1.31}
    persistence.save(path, persistence.build_update(_sensors(), odom, ('/odometry_node',)))
    first = open(path).read()
    persistence.save(path, persistence.build_update(_sensors(), odom, ('/odometry_node',)),
                     stamp=None)
    # Same values: same body (only the header stamp may differ)
    assert first.split('\n', 2)[2] == open(path).read().split('\n', 2)[2]

    # Second run: only the lidar, no odometry
    persistence.save(path, persistence.build_update({'lidar': _sensors(-1.6)['lidar']}))
    text = open(path).read()
    assert text.count('base_link_to_lidar') == 1
    assert text.count('odometry_node') == 1
    data = yaml.safe_load(text)
    ext = persistence.extrinsics(data)
    assert ext['lidar']['rotation_rpy'] == [0.0, 0.0, -1.6]
    assert ext['imu']['rotation_rpy'] == [0.01, -0.02, 0.0]
    assert data['odometry_node']['ros__parameters']['chi'] == 1.31


def test_missing_and_empty_files(tmp_path):
    assert persistence.load(str(tmp_path / 'none.yaml')) == {}
    (tmp_path / 'empty.yaml').write_text('')
    assert persistence.load(str(tmp_path / 'empty.yaml')) == {}
    assert persistence.extrinsics({}) == {}


def test_deep_merge_does_not_alias():
    base = {'a': {'b': [1, 2]}}
    merged = persistence.deep_merge(base, {'a': {'c': 3}})
    merged['a']['b'].append(9)
    assert base == {'a': {'b': [1, 2]}}
    assert merged['a']['c'] == 3
