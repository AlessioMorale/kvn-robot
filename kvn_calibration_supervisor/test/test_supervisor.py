"""FSM walk-through with rclpy: sensor callbacks and the FSM tick are driven by hand."""
import json
import math

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from kvn_calibration_supervisor import algorithms as alg  # noqa: E402
from kvn_calibration_supervisor.supervisor_node import CalibrationSupervisor, State  # noqa: E402

LIDAR_YAW = -math.pi / 2 + 0.02   # the true mounting; the URDF says -pi/2
IMU_MOUNT = (0.02, -0.01)
FLOOR = alg.rpy_to_matrix(0.03, -0.02, 0.0)


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    node = CalibrationSupervisor(parameter_overrides=[
        Parameter('steps', value=['imu', 'lidar']),
        Parameter('output_path', value=str(tmp_path / 'calibration_test.yaml')),
        Parameter('imu_samples', value=50),
        Parameter('imu_settle_time', value=0.0),
    ])
    for child, xyz, yaw in (('imu_link', (-0.05, -0.03, 0.025), 0.0),
                            ('lidar_attachment_link', (0.03, 0.0, 0.115), -math.pi / 2),
                            ('camera_attachment_link', (0.074, 0.0, 0.1), 0.0)):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        node.tf_buffer.set_transform_static(tf, 'test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def command(node, text):
    node._on_command(String(data=text))
    node._tick()


def imu_msg(stamp, heading, rate=0.0):
    base = FLOOR @ alg.rpy_to_matrix(0.0, 0.0, heading)
    f = (base @ alg.rpy_to_matrix(*IMU_MOUNT, 0.0)).T @ np.array([0.0, 0.0, 9.81])
    msg = Imu()
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(stamp), int((stamp % 1) * 1e9)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = f
    msg.angular_velocity.z = rate
    return msg


def wall_scan():
    n = 720
    msg = LaserScan()
    msg.header.frame_id = 'lidar_attachment_link'
    msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / n
    msg.range_min, msg.range_max = 0.05, 12.0
    ranges = []
    for i in range(n):
        a = msg.angle_min + i * msg.angle_increment + LIDAR_YAW
        ranges.append((1.0 - 0.03) / math.cos(a) if math.cos(a) > 0.1 else float('inf'))
    msg.ranges = ranges
    return msg


def test_full_session_and_save(node, tmp_path):
    published = []
    node.cmd_vel_pub.publish = published.append
    assert node.state == State.IDLE
    command(node, 'next')  # ignored outside a WAIT state
    assert node.state == State.IDLE and 'ignored' in node.message

    command(node, 'START')
    assert node.state == State.IMU_WAIT_POS_1  # odometry is not in this node's steps
    command(node, 'NEXT')
    assert node.state == State.IMU_SAMPLING_POS_1
    t = 100.0
    for _ in range(50):
        node._on_imu(imu_msg(t, 0.0, rate=0.001))
        t += 0.01
    node._tick()
    assert node.state == State.IMU_AUTONOMOUS_TURN
    assert node.results['imu']['gyro_bias'] == pytest.approx(0.001)

    # Gyro reports the turn; the node commands a CCW rotation until it reaches pi
    rate = 0.5
    while node.state == State.IMU_AUTONOMOUS_TURN:
        node._on_imu(imu_msg(t, 0.0, rate=rate + 0.001))
        t += 0.02
        node._tick()
        assert t < 120.0
    assert published and published[0].angular.z > 0.0
    assert published[-1].angular.z == 0.0  # stopped
    assert node.state == State.IMU_SAMPLING_POS_2
    for _ in range(50):
        node._on_imu(imu_msg(t, math.pi, rate=0.001))
        t += 0.01
    node._tick()
    assert node.state == State.LIDAR_WAIT
    imu = node.results['imu']
    assert imu['roll'] == pytest.approx(IMU_MOUNT[0], abs=1e-3)
    assert imu['pitch'] == pytest.approx(IMU_MOUNT[1], abs=1e-3)
    assert abs(imu['turn_gyro_deg'] - 180.0) < 2.0

    command(node, 'NEXT')
    assert node.state == State.LIDAR_SAMPLING
    for _ in range(10):
        node._on_scan(wall_scan())
    node._tick()
    assert node.state == State.REVIEW_AND_SAVE
    assert node.results['lidar']['yaw'] == pytest.approx(LIDAR_YAW, abs=math.radians(0.2))
    json.dumps(node._status())  # the status must serialise

    command(node, 'SAVE')
    assert node.state == State.IDLE and 'saved' in node.message
    data = yaml.safe_load(open(tmp_path / 'calibration_test.yaml'))
    ext = data['/**']['ros__parameters']['extrinsics']
    assert ext['base_link_to_imu']['rotation_rpy'][:2] == pytest.approx(IMU_MOUNT, abs=1e-3)
    assert ext['base_link_to_lidar']['rotation_rpy'][2] == pytest.approx(LIDAR_YAW, abs=4e-3)
    assert ext['base_link_to_lidar']['translation'] == pytest.approx([0.03, 0.0, 0.115])
    # Not calibrated this run: the URDF values
    assert ext['base_link_to_oakd']['rotation_rpy'] == pytest.approx([0.0, 0.0, 0.0])
    assert 'odometry_node' not in data


def test_skip_and_abort(node):
    published = []
    node.cmd_vel_pub.publish = published.append
    command(node, 'START')
    command(node, 'SKIP')
    assert node.state == State.LIDAR_WAIT and node.message == 'imu skipped'
    command(node, 'NEXT')
    assert node.state == State.LIDAR_SAMPLING
    command(node, 'ABORT')  # nothing running in the background: back to IDLE in the same tick
    assert node.state == State.IDLE and node.message == 'aborted' and node.results == {}
    assert published and published[-1].angular.z == 0.0
    assert node._sensor_subs == []
    command(node, 'START')
    assert node.state == State.IMU_WAIT_POS_1


def test_sampling_without_data_returns_to_wait(node):
    node.sensor_timeout = 0.0
    command(node, 'START')
    command(node, 'SKIP')
    command(node, 'NEXT')
    node._tick()
    assert node.state == State.LIDAR_WAIT
    assert 'no scan' in node.error


def test_moving_rover_fails_imu_sampling(node):
    command(node, 'START')
    command(node, 'NEXT')
    for i in range(50):
        node._on_imu(imu_msg(100.0 + 0.01 * i, 0.0, rate=0.3))
    node._tick()
    assert node.state == State.IMU_WAIT_POS_1
    assert 'moved' in node.error


def test_unknown_command(node):
    command(node, 'DANCE')
    assert node.state == State.IDLE and 'unknown command' in node.error
