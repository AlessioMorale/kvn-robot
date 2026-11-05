# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""calibration_supervisor: guided calibration of the wheel odometry and the sensor extrinsics.

Driven entirely by topics, so that it works from the Foxglove/Lichtblick publish and raw-message
panels without SSH:

  /calibration/command  std_msgs/String  START, NEXT, SKIP, ABORT, SAVE (case-insensitive)
  /calibration/status   std_msgs/String  JSON: state, prompt, accepted commands, progress,
                                         results, error (latched, and repeated every second)

Steps, each one waiting for NEXT (or SKIP) once the rover is in place:

  odometry  wheel_radius and chi (kvn_odom_calibration, run in this process)
  imu       IMU roll/pitch, sampled at two headings 180 deg apart to cancel the floor's tilt;
            the half turn is driven by the node
  lidar     lidar yaw, from a wall the rover is square to
  camera    OAK-D pitch/yaw, from the same wall's plane in the depth cloud vs the lidar's normal

then REVIEW_AND_SAVE shows the results and SAVE writes them to /etc/kvn/calibration_<host>.yaml.
Sensor topics are only subscribed while a step samples them.
"""
from enum import Enum
import json
import math
import queue
import signal
import threading
import time

import message_filters
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import String

from kvn_calibration_supervisor import algorithms as alg
from kvn_calibration_supervisor import persistence


class State(str, Enum):
    IDLE = 'IDLE'
    ODOM_WAIT = 'ODOM_WAIT'
    ODOM_CALIBRATING = 'ODOM_CALIBRATING'
    IMU_WAIT_POS_1 = 'IMU_WAIT_POS_1'
    IMU_SAMPLING_POS_1 = 'IMU_SAMPLING_POS_1'
    IMU_AUTONOMOUS_TURN = 'IMU_AUTONOMOUS_TURN'
    IMU_SAMPLING_POS_2 = 'IMU_SAMPLING_POS_2'
    LIDAR_WAIT = 'LIDAR_WAIT'
    LIDAR_SAMPLING = 'LIDAR_SAMPLING'
    CAMERA_WAIT = 'CAMERA_WAIT'
    CAMERA_SAMPLING = 'CAMERA_SAMPLING'
    REVIEW_AND_SAVE = 'REVIEW_AND_SAVE'
    ABORT_ROUTINE = 'ABORT_ROUTINE'


STEPS = ('odometry', 'imu', 'lidar', 'camera')
STEP_WAIT = {'odometry': State.ODOM_WAIT, 'imu': State.IMU_WAIT_POS_1,
             'lidar': State.LIDAR_WAIT, 'camera': State.CAMERA_WAIT}
STEP_OF = {
    State.ODOM_WAIT: 'odometry', State.ODOM_CALIBRATING: 'odometry',
    State.IMU_WAIT_POS_1: 'imu', State.IMU_SAMPLING_POS_1: 'imu',
    State.IMU_AUTONOMOUS_TURN: 'imu', State.IMU_SAMPLING_POS_2: 'imu',
    State.LIDAR_WAIT: 'lidar', State.LIDAR_SAMPLING: 'lidar',
    State.CAMERA_WAIT: 'camera', State.CAMERA_SAMPLING: 'camera',
}
NEXT_OF_WAIT = {State.ODOM_WAIT: State.ODOM_CALIBRATING,
                State.IMU_WAIT_POS_1: State.IMU_SAMPLING_POS_1,
                State.LIDAR_WAIT: State.LIDAR_SAMPLING,
                State.CAMERA_WAIT: State.CAMERA_SAMPLING}
MOVING = (State.ODOM_CALIBRATING, State.IMU_AUTONOMOUS_TURN)

PROMPTS = {
    State.IDLE: 'Calibration ready. Send START.',
    State.ODOM_WAIT: 'Odometry: place the rover in an open area with about 1.4 m free in front '
                     'and behind and walls in view in more than one direction. The rover drives '
                     'by itself for a few minutes. Send NEXT (or SKIP).',
    State.ODOM_CALIBRATING: 'Odometry: driving straight runs and spins. Keep clear; ABORT stops '
                            'the rover.',
    State.IMU_WAIT_POS_1: 'IMU: place the robot on a stationary surface with room to turn in '
                          'place, then send NEXT (or SKIP). Do not touch it until the step ends.',
    State.IMU_SAMPLING_POS_1: 'IMU: sampling at the first heading, do not touch the rover.',
    State.IMU_AUTONOMOUS_TURN: 'IMU: turning 180 deg in place. Keep clear; ABORT stops the rover.',
    State.IMU_SAMPLING_POS_2: 'IMU: sampling at the second heading, do not touch the rover.',
    State.LIDAR_WAIT: 'LiDAR: place the robot perfectly perpendicular to a flat wall, 1.0 m '
                      'away, then send NEXT (or SKIP).',
    State.LIDAR_SAMPLING: 'LiDAR: capturing scans of the wall.',
    State.CAMERA_WAIT: 'Camera: ensure the OAK-D Lite has a clear view of the wall (same '
                       'position as for the LiDAR), then send NEXT (or SKIP).',
    State.CAMERA_SAMPLING: 'Camera: capturing depth clouds of the wall.',
    State.REVIEW_AND_SAVE: 'Review the results, then send SAVE to write them or ABORT to '
                           'discard them.',
    State.ABORT_ROUTINE: 'Aborting: stopping the rover and clearing the session.',
}


def _accepted(state):
    if state == State.IDLE:
        return ['START']
    if state == State.ABORT_ROUTINE:
        return []
    if state == State.REVIEW_AND_SAVE:
        return ['SAVE', 'ABORT']
    if state in NEXT_OF_WAIT:
        return ['NEXT', 'SKIP', 'ABORT']
    return ['SKIP', 'ABORT']


class StepFailed(Exception):
    """A sampling step could not produce a result: back to its WAIT state with the reason."""


class CalibrationSupervisor(Node):

    def __init__(self, **kwargs):
        super().__init__('calibration_supervisor', **kwargs)
        declare = self.declare_parameter
        self.steps = [s for s in declare('steps', list(STEPS)).value if s in STEPS]
        output = declare('output_path', '').value
        self.output_path = output or persistence.default_path()
        self.base_frame = declare('base_frame', 'base_link').value
        # Frames whose pose in base_link is calibrated; the translations come from TF (URDF)
        self.sensor_frames = {
            'imu': declare('imu_frame', 'imu_link').value,
            'oakd': declare('camera_frame', 'camera_attachment_link').value,
            'lidar': declare('lidar_frame', 'lidar_attachment_link').value,
        }
        self.imu_topic = declare('imu_topic', '/imu').value
        self.scan_topic = declare('scan_topic', '/scan').value
        self.cloud_topic = declare('cloud_topic', '/oak/rgbd/points').value
        self.odom_topic = declare('odom_topic', '/odom').value
        self.fsm_rate = declare('fsm_rate', 20.0).value
        self.sensor_timeout = declare('sensor_timeout', 5.0).value

        # IMU
        self.imu_samples = declare('imu_samples', 500).value
        self.imu_settle_time = declare('imu_settle_time', 1.0).value  # before the 2nd sampling
        self.max_still_rate = declare('max_still_rate', 0.05).value   # [rad/s] "not moving"
        # Half turn: progress = w * gyro + (1 - w) * odom yaw (gyro only without /odom)
        self.turn_direction = 1.0 if declare('turn_direction', 1).value >= 0 else -1.0
        self.turn_gyro_weight = declare('turn_gyro_weight', 0.8).value
        self.turn_max_speed = declare('turn_max_speed', 0.6).value   # [rad/s]
        self.turn_min_speed = declare('turn_min_speed', 0.15).value
        self.turn_gain = declare('turn_gain', 1.5).value             # [1/s] slow-down near the end
        self.turn_tolerance = declare('turn_tolerance', 0.01).value  # [rad]
        self.turn_timeout = declare('turn_timeout', 30.0).value      # [s]

        # LiDAR wall line
        self.lidar_scans = declare('lidar_scans', 10).value
        self.lidar_window = declare('lidar_window', 0.5).value       # [rad] half width, forward
        self.lidar_max_range = declare('lidar_max_range', 3.0).value
        self.wall_distance = declare('wall_distance', 1.0).value     # [m] expected, warns if off
        self.ransac_threshold_2d = declare('ransac_threshold_2d', 0.015).value
        self.max_wall_angle = declare('max_wall_angle', 0.5).value   # [rad] vs expected normal
        self.ransac_iterations = declare('ransac_iterations', 2000).value

        # Camera wall plane
        self.camera_clouds = declare('camera_clouds', 3).value
        self.sync_slop = declare('sync_slop', 0.1).value             # [s] scan / cloud pairing
        self.camera_min_range = declare('camera_min_range', 0.3).value
        self.camera_max_range = declare('camera_max_range', 2.5).value
        self.camera_crop_h = declare('camera_crop_horizontal', 0.35).value  # [rad] half angles
        self.camera_crop_v = declare('camera_crop_vertical', 0.25).value
        self.camera_max_points = declare('camera_max_points', 20000).value
        self.ransac_threshold_3d = declare('ransac_threshold_3d', 0.02).value

        # Odometry step (kvn_odom_calibration, its parameters under this node name)
        self.odom_node_name = declare('odom_calibration_node', 'odom_calibration').value
        self.controller_node = declare('controller_node', '/skid_steer_controller').value
        self.odometry_node = declare('odometry_node', '/odometry_node').value

        self._lock = threading.Lock()
        self._commands = queue.SimpleQueue()
        self._rng = np.random.default_rng()
        self._fsm_group = MutuallyExclusiveCallbackGroup()
        self._sensor_group = MutuallyExclusiveCallbackGroup()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/calibration/status', latched)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/calibration/command', self._on_command, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, static_only=True)

        self._sensor_subs = []
        self._odom_thread = None
        self._odom_routine = None
        self._odom_outcome = None
        self._odom_outcome_override = None
        self._synchronizer = None
        self.state = None
        self._reset_session()
        self._enter(State.IDLE)
        self.create_timer(1.0 / self.fsm_rate, self._tick, callback_group=self._fsm_group)
        self.create_timer(1.0, self._publish_status, callback_group=self._fsm_group)
        self.get_logger().info(
            f'Calibration supervisor ready, steps {self.steps}, output {self.output_path}. '
            'Commands on /calibration/command, status on /calibration/status.')

    # ---- session ------------------------------------------------------------------------------

    def _reset_session(self):
        with self._lock:
            self.results = {}       # step -> computed values, shown in the status
            self.progress = {}
            self.error = None
            self.message = None
            self._clear_buffers()

    def _clear_buffers(self):
        self._imu_accel = []
        self._imu_gyro_z = []
        self._imu_pos_1 = None
        self._gyro_bias = 0.0
        self._gyro_yaw = 0.0
        self._last_gyro = None
        self._odom_yaw = None
        self._odom_turn = 0.0
        self._scans = []
        self._pairs = []
        self._sample_after = None
        self._turn_speed = 0.0

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self):
        return (self._now() - self._entered).nanoseconds * 1e-9

    # ---- commands and status ------------------------------------------------------------------

    def _on_command(self, msg):
        self._commands.put(msg.data.strip().upper())

    def _status(self):
        with self._lock:
            return {
                'state': self.state.value,
                'step': STEP_OF.get(self.state),
                'steps': self.steps,
                'prompt': PROMPTS[self.state],
                'accepted_commands': _accepted(self.state),
                'progress': self.progress,
                'results': self.results,
                'error': self.error,
                'message': self.message,
                'output_path': self.output_path,
                'stamp': time.time(),
            }

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(_sanitize(self._status()),
                                                       allow_nan=False)))

    def _set_progress(self, **values):
        with self._lock:
            self.progress = values

    # ---- transitions --------------------------------------------------------------------------

    def _enter(self, state, error=None, message=None):
        self._unsubscribe_sensors()
        previous = self.state
        with self._lock:
            self.state = state
            self.error = error
            self.message = message
            self.progress = {}
        self._entered = self._now()
        if error:
            self.get_logger().error(f'{previous.value if previous else "-"} -> {state.value}: '
                                    f'{error}')
        else:
            self.get_logger().info(f'{previous.value if previous else "-"} -> {state.value}'
                                   + (f': {message}' if message else ''))
        getattr(self, f'_enter_{state.value.lower()}', lambda: None)()
        self._publish_status()

    def _first_step_from(self, step_index):
        for step in STEPS[step_index:]:
            if step in self.steps:
                return STEP_WAIT[step]
        return State.REVIEW_AND_SAVE

    def _after(self, step):
        return self._first_step_from(STEPS.index(step) + 1)

    def _process_command(self, command):
        state = self.state
        if command not in ('START', 'NEXT', 'SKIP', 'ABORT', 'SAVE'):
            with self._lock:
                self.error = f'unknown command {command!r} (START, NEXT, SKIP, ABORT, SAVE)'
            return
        if command not in _accepted(state):
            with self._lock:
                self.message = f'{command} ignored in {state.value}'
            self.get_logger().warning(self.message)
            return
        if command == 'ABORT':
            self._enter(State.ABORT_ROUTINE)
        elif command == 'START':
            self._reset_session()
            self._enter(self._first_step_from(0))
        elif command == 'NEXT':
            self._enter(NEXT_OF_WAIT[state])
        elif command == 'SKIP':
            self._skip(state)
        elif command == 'SAVE':
            self._save()

    def _skip(self, state):
        step = STEP_OF[state]
        with self._lock:
            self.results.pop(step, None)
        if state == State.ODOM_CALIBRATING:
            # The routine drives the rover: stop it first, the tick moves on once it has ended
            self._odom_outcome_override = ('skip', None)
            self._odom_routine_abort()
            with self._lock:
                self.message = 'skipping odometry: stopping the rover'
            return
        self._stop_robot()
        self._enter(self._after(step), message=f'{step} skipped')

    # ---- FSM loop -----------------------------------------------------------------------------

    def _tick(self):
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self.get_logger().info(f'Command {command} in {self.state.value}')
            self._process_command(command)
        state = self.state
        try:
            getattr(self, f'_tick_{state.value.lower()}', lambda: None)()
        except StepFailed as e:
            self._stop_robot()
            self._enter(STEP_WAIT[STEP_OF[state]], error=str(e))
        except Exception as e:  # noqa: BLE001 - never leave the rover driving on a bug
            self._stop_robot()
            self.get_logger().error(f'Unexpected error in {state.value}: {e!r}')
            self._enter(STEP_WAIT[STEP_OF[state]] if state in STEP_OF else State.IDLE,
                        error=f'internal error: {e!r}')

    def _check_timeout(self, received, what):
        if received == 0 and self._elapsed() > self.sensor_timeout:
            raise StepFailed(f'no {what} within {self.sensor_timeout:.0f} s')

    # ---- sensors ------------------------------------------------------------------------------

    def _subscribe(self, msg_type, topic, callback):
        self._sensor_subs.append(self.create_subscription(
            msg_type, topic, callback, qos_profile_sensor_data, callback_group=self._sensor_group))

    def _unsubscribe_sensors(self):
        for sub in self._sensor_subs:
            self.destroy_subscription(sub)
        self._sensor_subs = []
        self._synchronizer = None

    def _on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        rate = msg.angular_velocity.z
        with self._lock:
            if self._last_gyro is not None:
                dt = stamp - self._last_gyro[0]
                if 0.0 < dt < 0.5:
                    self._gyro_yaw += (0.5 * (rate + self._last_gyro[1]) - self._gyro_bias) * dt
            self._last_gyro = (stamp, rate)
            if self._sample_after is not None and self._now() >= self._sample_after:
                a = msg.linear_acceleration
                self._imu_accel.append((a.x, a.y, a.z))
                self._imu_gyro_z.append(rate)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            if self._odom_yaw is not None:
                self._odom_turn += alg.wrap_angle(yaw - self._odom_yaw)
            self._odom_yaw = yaw

    def _on_scan(self, msg):
        with self._lock:
            self._scans.append(msg)

    def _on_scan_cloud(self, scan, cloud):
        with self._lock:
            self._pairs.append((scan, cloud))

    def _lookup(self, target, source):
        """(translation, rotation matrix) of source in target, from the static TF tree."""
        tf = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=1.0))
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), alg.quaternion_to_matrix(q.x, q.y, q.z, q.w)

    def _sensor_pose(self, sensor):
        """Current (URDF) translation and rpy of a sensor frame in base_link, or None."""
        try:
            t, r = self._lookup(self.base_frame, self.sensor_frames[sensor])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f'No TF {self.base_frame} -> {self.sensor_frames[sensor]}: {e}')
            return None
        return t, alg.matrix_to_rpy(r)

    # ---- odometry -----------------------------------------------------------------------------

    def _enter_odom_calibrating(self):
        self._odom_outcome = None
        self._odom_outcome_override = None
        self._odom_thread = threading.Thread(target=self._odom_worker, daemon=True)
        self._odom_thread.start()

    def _odom_worker(self):
        # Imported here: only this step needs kvn_odom_calibration
        from kvn_odom_calibration.odom_calibration_node import CalibrationAborted, OdomCalibration
        outcome = ('error', 'did not run')
        routine = None
        executor = self.executor
        try:
            # Launch dict parameters only reach this node: hand use_sim_time down explicitly
            routine = OdomCalibration(
                node_name=self.odom_node_name,
                parameter_overrides=[self.get_parameter('use_sim_time')])
            routine.on_segment = lambda n, total, s: self._set_progress(
                run=n, runs=total, kind=s.kind, valid=s.valid, note=s.note or None)
            with self._lock:
                self._odom_routine = routine
            executor.add_node(routine)
            outcome = ('ok', routine.run())
        except CalibrationAborted as e:
            outcome = ('error', str(e))
        except Exception as e:  # noqa: BLE001 - reported in the status, the FSM stays usable
            outcome = ('error', f'{e!r}')
        finally:
            if routine is not None:
                routine.stop_robot()
                routine.restore_parameters()
                executor.remove_node(routine)
                routine.destroy_node()
            with self._lock:
                self._odom_routine = None
                self._odom_outcome = outcome

    def _odom_routine_abort(self):
        with self._lock:
            routine = self._odom_routine
        if routine is not None:
            routine.request_abort()

    def _odom_running(self):
        return self._odom_thread is not None and self._odom_thread.is_alive()

    def _tick_odom_calibrating(self):
        if self._odom_running():
            return
        with self._lock:
            outcome = self._odom_outcome_override or self._odom_outcome
        if outcome[0] == 'skip':
            self._enter(self._after('odometry'), message='odometry skipped')
        elif outcome[0] == 'error':
            raise StepFailed(f'odometry calibration failed: {outcome[1]}')
        else:
            with self._lock:
                self.results['odometry'] = outcome[1]
            self._enter(self._after('odometry'), message=f'odometry: {outcome[1]}')

    # ---- IMU ----------------------------------------------------------------------------------

    def _start_imu_sampling(self, delay):
        with self._lock:
            self._imu_accel, self._imu_gyro_z = [], []
            self._sample_after = self._now() + Duration(seconds=delay)
        self._subscribe(Imu, self.imu_topic, self._on_imu)

    def _collect_imu(self):
        with self._lock:
            n = len(self._imu_accel)
            if n < self.imu_samples:
                self.progress = {'samples': n, 'of': self.imu_samples}
                accel = None
            else:
                accel = np.array(self._imu_accel[:self.imu_samples])
                gyro = np.array(self._imu_gyro_z[:self.imu_samples])
                self._sample_after = None
        if accel is None:
            settling = self._now() < self._sample_after
            waited = self._elapsed() - self.imu_settle_time
            if n == 0 and not settling and waited > self.sensor_timeout:
                raise StepFailed(f'no IMU data on {self.imu_topic}')
            return None
        rate = float(np.mean(np.abs(gyro)))
        if rate > self.max_still_rate + abs(self._gyro_bias):
            raise StepFailed(f'the rover moved while sampling (mean |yaw rate| {rate:.3f} rad/s)')
        return accel, gyro

    def _enter_imu_sampling_pos_1(self):
        with self._lock:
            self._gyro_bias = 0.0
        self._start_imu_sampling(0.0)

    def _tick_imu_sampling_pos_1(self):
        data = self._collect_imu()
        if data is None:
            return
        accel, gyro = data
        roll, pitch = alg.accel_roll_pitch(accel)
        with self._lock:
            self._imu_pos_1 = (roll, pitch)
            self._gyro_bias = float(np.mean(gyro))  # stationary: the whole yaw rate is bias
            self.results['imu'] = {'roll_0': roll, 'pitch_0': pitch, 'gyro_bias': self._gyro_bias}
        self._enter(State.IMU_AUTONOMOUS_TURN)

    def _enter_imu_autonomous_turn(self):
        with self._lock:
            self._gyro_yaw = 0.0
            self._last_gyro = None
            self._odom_yaw = None
            self._odom_turn = 0.0
            self._turn_speed = 0.0
        self._subscribe(Imu, self.imu_topic, self._on_imu)
        self._subscribe(Odometry, self.odom_topic, self._on_odom)

    def _turn_angle(self):
        with self._lock:
            gyro = self._gyro_yaw if self._last_gyro is not None else None
            odom = self._odom_turn if self._odom_yaw is not None else None
        if gyro is None:
            return odom, gyro, odom
        if odom is None:
            return gyro, gyro, odom
        w = self.turn_gyro_weight
        return w * gyro + (1.0 - w) * odom, gyro, odom

    def _tick_imu_autonomous_turn(self):
        angle, gyro, odom = self._turn_angle()
        if angle is None:
            if self._elapsed() > self.sensor_timeout:
                raise StepFailed(f'no {self.imu_topic} or {self.odom_topic} to measure the turn')
            return
        remaining = math.pi - self.turn_direction * angle
        self._set_progress(angle_deg=math.degrees(angle),
                           gyro_deg=None if gyro is None else math.degrees(gyro),
                           odom_deg=None if odom is None else math.degrees(odom))
        if remaining <= self.turn_tolerance:
            self._stop_robot()
            self._enter(State.IMU_SAMPLING_POS_2)
            return
        if self._elapsed() > self.turn_timeout:
            raise StepFailed(f'the half turn did not complete within {self.turn_timeout:.0f} s '
                             f'(at {math.degrees(angle):.0f} deg; stuck?)')
        speed = min(self.turn_max_speed, max(self.turn_min_speed, self.turn_gain * remaining))
        self._publish_twist(angular=self.turn_direction * speed)

    def _enter_imu_sampling_pos_2(self):
        # Keep integrating the turn through the settling, to report the final angle
        self._subscribe(Odometry, self.odom_topic, self._on_odom)
        self._start_imu_sampling(self.imu_settle_time)

    def _tick_imu_sampling_pos_2(self):
        data = self._collect_imu()
        if data is None:
            return
        accel, _ = data
        roll, pitch = alg.accel_roll_pitch(accel)
        bias = alg.floor_compensated_bias(self._imu_pos_1, (roll, pitch))
        _, gyro, odom = self._turn_angle()
        with self._lock:
            self.results['imu'].update({
                'roll_180': roll, 'pitch_180': pitch,
                'roll': bias.roll, 'pitch': bias.pitch,
                'floor_roll': bias.floor_roll, 'floor_pitch': bias.floor_pitch,
                'turn_gyro_deg': None if gyro is None else math.degrees(gyro),
                'turn_odom_deg': None if odom is None else math.degrees(odom),
            })
        self._enter(self._after('imu'), message=(
            f'IMU roll {math.degrees(bias.roll):+.2f} deg, pitch {math.degrees(bias.pitch):+.2f} '
            f'deg (floor {math.degrees(bias.floor_roll):+.2f} / '
            f'{math.degrees(bias.floor_pitch):+.2f} deg)'))

    # ---- LiDAR --------------------------------------------------------------------------------

    def _enter_lidar_sampling(self):
        with self._lock:
            self._scans = []
        self._subscribe(LaserScan, self.scan_topic, self._on_scan)

    def _lidar_frame_yaw(self, frame):
        """Current yaw of the scan frame in base_link (URDF), for the forward window."""
        try:
            _, r = self._lookup(self.base_frame, frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            raise StepFailed(f'no TF {self.base_frame} -> {frame}: {e}') from e
        return alg.matrix_to_rpy(r)[2]

    def _fit_wall_line(self, scans, frame_yaw):
        """Wall line in front of the rover, in the scan frame, from several scans."""
        center = -frame_yaw  # base_link +x seen from the lidar
        points = np.vstack([alg.scan_to_points(
            s.ranges, s.angle_min, s.angle_increment, s.range_min,
            min(s.range_max, self.lidar_max_range), center, self.lidar_window) for s in scans])
        try:
            return alg.ransac_hyperplane(
                points, (math.cos(center), math.sin(center)), self.max_wall_angle,
                self.ransac_threshold_2d, self.ransac_iterations, rng=self._rng)
        except ValueError as e:
            raise StepFailed(f'no wall in front of the lidar: {e}') from e

    def _tick_lidar_sampling(self):
        with self._lock:
            scans = list(self._scans)
            self.progress = {'scans': len(scans), 'of': self.lidar_scans}
        self._check_timeout(len(scans), f'scan on {self.scan_topic}')
        if len(scans) < self.lidar_scans:
            return
        self._unsubscribe_sensors()
        frame = scans[0].header.frame_id
        current_yaw = self._lidar_frame_yaw(frame)
        fit = self._fit_wall_line(scans[:self.lidar_scans], current_yaw)
        yaw = alg.lidar_yaw_from_wall(fit)
        result = {'yaw': yaw, 'urdf_yaw': current_yaw,
                  'correction_deg': math.degrees(alg.wrap_angle(yaw - current_yaw)),
                  'wall_distance': fit.distance, 'rmse': fit.rmse,
                  'inlier_ratio': fit.inlier_ratio, 'frame': frame}
        warning = None
        if abs(fit.distance - self.wall_distance) > 0.3:
            warning = (f'wall at {fit.distance:.2f} m from the lidar, expected about '
                       f'{self.wall_distance:.1f} m')
            result['warning'] = warning
        with self._lock:
            self.results['lidar'] = result
        self._enter(self._after('lidar'), message=(
            f'lidar yaw {math.degrees(yaw):+.2f} deg ({result["correction_deg"]:+.2f} deg vs URDF)'
            + (f'; {warning}' if warning else '')))

    # ---- camera -------------------------------------------------------------------------------

    def _enter_camera_sampling(self):
        with self._lock:
            self._pairs = []
        scan_sub = message_filters.Subscriber(self, LaserScan, self.scan_topic,
                                              qos_profile=qos_profile_sensor_data,
                                              callback_group=self._sensor_group)
        cloud_sub = message_filters.Subscriber(self, PointCloud2, self.cloud_topic,
                                               qos_profile=qos_profile_sensor_data,
                                               callback_group=self._sensor_group)
        self._sensor_subs += [scan_sub.sub, cloud_sub.sub]
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, cloud_sub], queue_size=10, slop=self.sync_slop)
        self._synchronizer.registerCallback(self._on_scan_cloud)

    def _cloud_points(self, cloud, frame):
        """Wall-facing points of a cloud in the camera mount frame (x forward)."""
        xyz = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        try:
            t, r = self._lookup(frame, cloud.header.frame_id)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            raise StepFailed(f'no TF {frame} -> {cloud.header.frame_id}: {e}') from e
        xyz = xyz @ r.T + t
        return alg.crop_forward(xyz, self.camera_min_range, self.camera_max_range,
                                self.camera_crop_h, self.camera_crop_v)

    def _tick_camera_sampling(self):
        with self._lock:
            pairs = list(self._pairs)
            self.progress = {'clouds': len(pairs), 'of': self.camera_clouds}
        self._check_timeout(len(pairs), f'synchronized {self.scan_topic} + {self.cloud_topic}')
        if len(pairs) < self.camera_clouds:
            return
        self._unsubscribe_sensors()
        pairs = pairs[:self.camera_clouds]

        # Wall normal in base_link from the lidar: calibrated yaw of this session, else the URDF
        scans = [scan for scan, _ in pairs]
        lidar_frame = scans[0].header.frame_id
        urdf_lidar_yaw = self._lidar_frame_yaw(lidar_frame)
        lidar_yaw = self.results.get('lidar', {}).get('yaw', urdf_lidar_yaw)
        line = self._fit_wall_line(scans, urdf_lidar_yaw)
        wall_angle = alg.wrap_angle(alg.wall_normal_angle(line) + lidar_yaw)
        normal_base = np.array([math.cos(wall_angle), math.sin(wall_angle), 0.0])

        mount = self.sensor_frames['oakd']
        points = np.vstack([self._cloud_points(cloud, mount) for _, cloud in pairs])
        points = alg.subsample(points, self.camera_max_points, self._rng)
        pose = self._sensor_pose('oakd')
        if pose is None:
            raise StepFailed(f'no TF {self.base_frame} -> {mount}')
        urdf_rpy = pose[1]
        expected = alg.rpy_to_matrix(*urdf_rpy).T @ normal_base  # in the mount frame
        try:
            plane = alg.ransac_hyperplane(points, expected, self.max_wall_angle,
                                          self.ransac_threshold_3d, self.ransac_iterations,
                                          rng=self._rng)
        except ValueError as e:
            raise StepFailed(f'no wall plane in the depth cloud: {e}') from e
        pitch, yaw = alg.camera_pitch_yaw(plane.normal, normal_base)
        result = {'pitch': pitch, 'yaw': yaw, 'urdf_pitch': urdf_rpy[1], 'urdf_yaw': urdf_rpy[2],
                  'pitch_correction_deg': math.degrees(alg.wrap_angle(pitch - urdf_rpy[1])),
                  'yaw_correction_deg': math.degrees(alg.wrap_angle(yaw - urdf_rpy[2])),
                  'wall_normal_base_deg': math.degrees(wall_angle),
                  'wall_distance': plane.distance, 'rmse': plane.rmse,
                  'inlier_ratio': plane.inlier_ratio, 'points': int(points.shape[0])}
        with self._lock:
            self.results['camera'] = result
        self._enter(self._after('camera'), message=(
            f'camera pitch {math.degrees(pitch):+.2f} deg, yaw {math.degrees(yaw):+.2f} deg'))

    # ---- review and save ----------------------------------------------------------------------

    def _enter_review_and_save(self):
        extrinsics = self._extrinsics()
        with self._lock:
            self.progress = {'extrinsics': extrinsics}

    def _extrinsics(self):
        """Values to write: computed this session, else the saved file, else the URDF."""
        rotations = {}
        imu = self.results.get('imu', {})
        if 'roll' in imu:
            rotations['imu'] = {0: imu['roll'], 1: imu['pitch']}
        if 'lidar' in self.results:
            rotations['lidar'] = {2: self.results['lidar']['yaw']}
        if 'camera' in self.results:
            rotations['oakd'] = {1: self.results['camera']['pitch'],
                                 2: self.results['camera']['yaw']}
        try:
            saved = persistence.extrinsics(persistence.load(self.output_path))
        except (OSError, ValueError) as e:
            self.get_logger().warning(f'Ignoring the saved calibration: {e}')
            saved = {}
        entries = {}
        for sensor in persistence.SENSORS:
            pose = self._sensor_pose(sensor)
            entry = {}
            if pose is not None:
                entry = {'translation': list(pose[0]), 'rotation_rpy': list(pose[1])}
            if sensor in saved and 'rotation_rpy' in saved[sensor]:
                entry['rotation_rpy'] = list(saved[sensor]['rotation_rpy'])
            if sensor in rotations:
                rpy = list(entry.get('rotation_rpy', [0.0, 0.0, 0.0]))
                for axis, value in rotations[sensor].items():
                    rpy[axis] = value
                entry['rotation_rpy'] = rpy
            if entry:
                entries[sensor] = entry
        return entries

    def _save(self):
        odometry = self.results.get('odometry')
        update = persistence.build_update(self._extrinsics(), odometry,
                                          (self.controller_node, self.odometry_node))
        try:
            persistence.save(self.output_path, update)
        except (OSError, ValueError) as e:
            with self._lock:
                self.error = f'could not write {self.output_path}: {e}'
            self.get_logger().error(self.error)
            self._publish_status()
            return
        self.get_logger().info(f'Calibration written to {self.output_path}: {self.results}')
        results = self.results
        self._reset_session()
        with self._lock:
            self.results = results  # keep them visible until the next START
        self._enter(State.IDLE, message=f'saved to {self.output_path}')

    # ---- abort --------------------------------------------------------------------------------

    def _enter_abort_routine(self):
        self._odom_routine_abort()
        self._stop_robot()
        self._reset_session()

    def _tick_abort_routine(self):
        self._stop_robot()
        if self._odom_running():
            return  # the odometry routine stops the rover and restores its parameters first
        self._enter(State.IDLE, message='aborted')

    # ---- motion -------------------------------------------------------------------------------

    def _publish_twist(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)

    def _stop_robot(self):
        # The odometry routine owns /cmd_vel while it runs
        if not self._odom_running():
            for _ in range(3):
                self._publish_twist()

    def shutdown(self, executor):
        """Stops the rover; lets a running odometry step restore the drive nodes' parameters."""
        self._odom_routine_abort()
        deadline = time.monotonic() + 10.0
        # Its parameter calls complete through the executor, which has stopped spinning
        while self._odom_running() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if self.state in MOVING or self.state == State.ABORT_ROUTINE:
            for _ in range(3):
                self._publish_twist()


def _sanitize(value):
    """JSON-safe copy: numpy to Python, NaN/inf to null (strict JSON for Foxglove)."""
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_sanitize(v) for v in value]
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return str(value)


def _interrupt(signum, frame):
    # Once only: ros2 run / systemd may signal again, which would cut the cleanup short
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    # Own SIGINT / SIGTERM (systemctl stop) handling: the context must still be valid to stop
    # the rover and let the odometry step restore the drive nodes' parameters
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = CalibrationSupervisor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted, stopping')
    finally:
        node.shutdown(executor)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
