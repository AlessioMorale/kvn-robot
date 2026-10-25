# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Automatic odometry calibration: drives the rover and fits wheel_radius and chi.

The rover drives straight runs (alternately forward and back) and spins in place (alternately
counter-clockwise and clockwise). For each run it integrates the wheel rotation from /joint_states
and measures the true motion between two stationary lidar scans, taken before and after the run,
by scan matching (ICP). The integrated gyro is the initial guess for the spins, which lets the scan
match resolve whole turns. Straight runs give wheel_radius, spins then give chi.

reference:=tf reads the motion from a TF frame instead (e.g. map from slam_toolbox, or a simulator
ground-truth frame), see README.md for why scan matching is the default.
"""
import csv
from datetime import datetime
import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
import tf2_ros
import yaml

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, LaserScan

from kvn_odom_calibration import fitting
from kvn_odom_calibration.fitting import SPIN, STRAIGHT, Segment
from kvn_odom_calibration.scan_matching import (
    icp, scan_to_points, se2, se2_params, transform_points, voxel_downsample, wrap_angle)


class CalibrationAborted(Exception):
    pass


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCalibration(Node):

    def __init__(self):
        super().__init__('odom_calibration')
        declare = self.declare_parameter
        self.base_frame = declare('base_frame', 'base_link').value
        self.left_joints = declare('left_joints', ['front_left_joint', 'back_left_joint']).value
        self.right_joints = declare('right_joints', ['front_right_joint', 'back_right_joint']).value
        # Nodes whose wheel_radius / track_width / chi are read (and written with apply_results)
        self.controller_node = declare('controller_node', '/skid_steer_controller').value
        self.odometry_node = declare('odometry_node', '/odometry_node').value
        # 'scan': ICP between stationary scans; 'tf': reference_frame -> base_frame from TF
        self.reference = declare('reference', 'scan').value
        self.reference_frame = declare('reference_frame', 'map').value
        self.use_imu = declare('use_imu', True).value

        self.straight_distance = declare('straight_distance', 1.0).value
        # Shorter runs are used where the lidar sees less free space; below this the run aborts
        self.min_straight_distance = declare('min_straight_distance', 0.4).value
        self.straight_runs = declare('straight_runs', 4).value
        self.linear_speed = declare('linear_speed', 0.15).value
        self.spin_angle = declare('spin_angle', 2.0 * math.pi).value
        self.spin_runs = declare('spin_runs', 4).value
        self.angular_speed = declare('angular_speed', 0.6).value
        self.settle_time = declare('settle_time', 1.5).value
        self.start_delay = declare('start_delay', 3.0).value

        self.max_scan_range = declare('max_scan_range', 8.0).value
        self.voxel_size = declare('voxel_size', 0.03).value
        self.icp_max_correspondence = declare('icp_max_correspondence', 0.3).value
        self.min_inlier_ratio = declare('min_inlier_ratio', 0.6).value
        self.min_observability = declare('min_observability', 0.03).value
        # A spin whose scan-matched yaw is this far from the gyro is a false match [rad]
        self.max_gyro_disagreement = declare('max_gyro_disagreement', math.radians(15.0)).value

        # Stop before starting a run if the lidar sees something in the way
        self.check_clearance = declare('check_clearance', True).value
        self.robot_half_length = declare('robot_half_length', 0.09).value
        self.robot_half_width = declare('robot_half_width', 0.07).value
        self.clearance_margin = declare('clearance_margin', 0.25).value

        # Slip compensation reacts to /odom and would fight the commanded wheel speeds
        self.disable_slip_compensation = declare('disable_slip_compensation', True).value
        self.apply_results = declare('apply_results', False).value
        self.output_dir = os.path.expanduser(declare('output_dir', '~/.ros/odom_calibration').value)

        self._lock = threading.Lock()
        self._phi_left = self._phi_right = 0.0
        self._last_wheels = None   # (stamp, left, right) wheel speeds [rad/s]
        self._gyro_raw = 0.0       # integrated raw gyro since the last reset [rad]
        self._gyro_time = 0.0      # time integrated since the last reset [s]
        self._last_gyro = None     # (stamp, rate)
        self._gyro_bias = 0.0
        self._bias_sum = 0.0
        self._bias_count = 0
        self._scan = None
        self._imu_seen = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 50)
        self.create_subscription(Imu, 'imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        # Scan matching only needs the static base -> lidar transform
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, static_only=self.reference != 'tf')
        self._restore = []  # (client, [Parameter]) to set back on exit

    # ---- callbacks ----------------------------------------------------------------------------

    def _on_joint_states(self, msg):
        speeds = dict(zip(msg.name, msg.velocity))
        left = [speeds[j] for j in self.left_joints if j in speeds]
        right = [speeds[j] for j in self.right_joints if j in speeds]
        if not left or not right:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        left, right = sum(left) / len(left), sum(right) / len(right)
        with self._lock:
            if self._last_wheels is not None:
                last_stamp, last_left, last_right = self._last_wheels
                dt = stamp - last_stamp
                if 0.0 < dt < 0.5:
                    self._phi_left += 0.5 * (left + last_left) * dt
                    self._phi_right += 0.5 * (right + last_right) * dt
            self._last_wheels = (stamp, left, right)

    def _on_imu(self, msg):
        # The IMU is mounted Z-up, so angular_velocity.z is the yaw rate
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        rate = msg.angular_velocity.z
        with self._lock:
            self._imu_seen = True
            if self._last_gyro is not None:
                dt = stamp - self._last_gyro[0]
                if 0.0 < dt < 0.5:
                    self._gyro_raw += 0.5 * (rate + self._last_gyro[1]) * dt
                    self._gyro_time += dt
            self._last_gyro = (stamp, rate)
            self._bias_sum += rate
            self._bias_count += 1

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    # ---- helpers ------------------------------------------------------------------------------

    def _sleep(self, seconds):
        """Sleep on the node clock, so that it also works with use_sim_time."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end:
            time.sleep(0.01)

    def _wait_for(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise CalibrationAborted(f'timed out waiting for {what}')
            time.sleep(0.05)

    def _wait_future(self, future, timeout, what):
        self._wait_for(future.done, timeout, what)
        if future.exception() is not None:
            raise CalibrationAborted(f'{what} failed: {future.exception()}')
        return future.result()

    def _reset_integrators(self):
        with self._lock:
            self._phi_left = self._phi_right = 0.0
            self._gyro_raw = self._gyro_time = 0.0

    def _read_integrators(self):
        with self._lock:
            gyro = self._gyro_raw - self._gyro_bias * self._gyro_time if self.use_imu else math.nan
            return self._phi_left, self._phi_right, gyro

    def _publish(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        for _ in range(5):
            self._publish()
            time.sleep(0.02)

    def _settle(self):
        """Stand still for settle_time and re-estimate the gyro bias meanwhile."""
        self.stop_robot()
        with self._lock:
            self._bias_sum, self._bias_count = 0.0, 0
        self._sleep(self.settle_time)
        with self._lock:
            if self._bias_count > 0:
                self._gyro_bias = self._bias_sum / self._bias_count

    def _fresh_scan(self):
        """First scan stamped after now, i.e. taken while the rover stands still."""
        after = self.get_clock().now()
        with self._lock:
            self._scan = None

        def ready():
            with self._lock:
                return self._scan is not None and Time.from_msg(self._scan.header.stamp) >= after
        self._wait_for(ready, 5.0, 'a fresh /scan')
        with self._lock:
            return self._scan

    def _scan_points(self, scan):
        points = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, min(scan.range_max, self.max_scan_range))
        return voxel_downsample(points, self.voxel_size)

    def _lookup_se2(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=2.0))
        t = tf.transform.translation
        return se2(t.x, t.y, _yaw_from_quaternion(tf.transform.rotation))

    # ---- parameters of the drive nodes --------------------------------------------------------

    def _get_params(self, node_name, names):
        client = AsyncParameterClient(self, node_name)
        if not client.wait_for_services(timeout_sec=5.0):
            self.get_logger().warning(f'{node_name} not found, its parameters are left alone')
            return None, None
        response = self._wait_future(client.get_parameters(names), 5.0, f'{node_name} get_parameters')
        return client, {n: parameter_value_to_python(v) for n, v in zip(names, response.values)}

    def _set_params(self, client, params, what):
        response = self._wait_future(client.set_parameters(params), 5.0, what)
        for p, result in zip(params, response.results):
            if not result.successful:
                self.get_logger().warning(f'{what}: {p.name} rejected: {result.reason}')

    def _read_kinematics(self):
        names = ['wheel_radius', 'track_width', 'chi']
        self.controller_client, controller = self._get_params(
            self.controller_node, names + ['slip_compensation_enabled'])
        self.odometry_client, odometry = self._get_params(self.odometry_node, names)
        # The odometry node's values are the ones /odom is computed with, i.e. what is calibrated
        source = odometry or controller
        if source is None:
            raise CalibrationAborted(
                f'neither {self.controller_node} nor {self.odometry_node} answered parameter requests')
        self.wheel_radius, self.track_width, self.chi = (float(source[n]) for n in names)
        if controller and odometry and any(
                abs(controller[n] - odometry[n]) > 1e-9 for n in names):
            self.get_logger().warning(
                f'{self.controller_node} and {self.odometry_node} disagree on the kinematics: '
                f'{ {n: controller[n] for n in names} } vs { {n: odometry[n] for n in names} }. '
                f'Fitting against {self.odometry_node}; the results set all three on both.')
        self.get_logger().info(
            f'Current kinematics: wheel_radius={self.wheel_radius:.5f} '
            f'track_width={self.track_width:.5f} chi={self.chi:.4f}')
        if controller and self.disable_slip_compensation and controller['slip_compensation_enabled']:
            self._set_params(self.controller_client,
                             [Parameter('slip_compensation_enabled', value=False)],
                             'disable slip compensation')
            self._restore.append(
                (self.controller_client, [Parameter('slip_compensation_enabled', value=True)]))
            self.get_logger().info('Slip compensation disabled for the calibration')

    def restore_parameters(self):
        for client, params in self._restore:
            try:
                self._set_params(client, params, 'restore parameters')
            except CalibrationAborted as e:
                self.get_logger().error(str(e))
        self._restore = []

    # ---- calibration --------------------------------------------------------------------------

    def _straight_target(self, direction, points_base):
        """Run length: straight_distance, shortened to the free space ahead; aborts below the minimum."""
        if not self.check_clearance or points_base is None or points_base.shape[0] == 0:
            return self.straight_distance
        x, y = points_base[:, 0], points_base[:, 1]
        ahead = direction * x
        in_path = (ahead > 0.0) & (np.abs(y) < self.robot_half_width + self.clearance_margin / 2.0)
        nearest = float(np.min(ahead[in_path])) if np.any(in_path) else math.inf
        free = nearest - self.robot_half_length - self.clearance_margin
        if free < self.min_straight_distance:
            raise CalibrationAborted(
                f'only {max(free, 0.0):.2f} m free {"ahead of" if direction > 0 else "behind"} the rover '
                f'(obstacle {nearest:.2f} m from base_link, margin {self.clearance_margin:.2f} m), '
                f'min_straight_distance is {self.min_straight_distance:.2f} m: move the rover to an '
                'open area')
        if free < self.straight_distance:
            self.get_logger().warning(
                f'Only {free:.2f} m free {"ahead" if direction > 0 else "behind"}: this straight run '
                f'is shortened from {self.straight_distance:.2f} m to {free:.2f} m')
        return min(self.straight_distance, free)

    def _check_spin_clearance(self, points_base):
        if not self.check_clearance or points_base is None or points_base.shape[0] == 0:
            return
        radius = math.hypot(self.robot_half_length, self.robot_half_width)
        dist = np.hypot(points_base[:, 0], points_base[:, 1])
        if np.any(dist < radius + self.clearance_margin / 2.0):
            raise CalibrationAborted(
                f'obstacle {float(dist.min()):.2f} m from base_link, closer than the spin radius '
                f'({radius:.2f} m + {self.clearance_margin / 2.0:.2f} m margin): move the rover away '
                'from walls')

    def _capture(self):
        """Reference state while standing still: (laser points, reference pose or None)."""
        points = pose = None
        if self.reference == 'scan' or self.check_clearance:
            points = self._scan_points(self._fresh_scan())
        if self.reference == 'tf':
            pose = self._lookup_se2(self.reference_frame, self.base_frame)
        return points, pose

    def _measure(self, start, end, guess):
        """Rover motion start -> end in the start base frame, unwrapped around the guess yaw."""
        guess_yaw = guess[2]
        if self.reference == 'tf':
            x, y, yaw = se2_params(np.linalg.inv(start[1]) @ end[1])
            quality = {}
        else:
            # ICP runs in the laser frame: T_laser = T_bl^-1 * T_base * T_bl
            T_bl = self.base_to_laser
            T_bl_inv = np.linalg.inv(T_bl)
            initial = T_bl_inv @ se2(guess[0], guess[1], wrap_angle(guess_yaw)) @ T_bl
            result = icp(end[0], start[0], initial=initial,
                         max_correspondence=self.icp_max_correspondence)
            x, y, yaw = se2_params(T_bl @ result.transform @ T_bl_inv)
            quality = {'rmse': result.rmse, 'inlier_ratio': result.inlier_ratio,
                       'observability': result.observability}
        return x, y, guess_yaw + wrap_angle(yaw - guess_yaw), quality

    def _run_segment(self, kind, direction):
        self._settle()
        start = self._capture()
        points_base = None if start[0] is None else transform_points(self.base_to_laser, start[0])
        if kind == STRAIGHT:
            target = self._straight_target(direction, points_base)
        else:
            self._check_spin_clearance(points_base)
        self._reset_integrators()

        if kind == STRAIGHT:
            speed = self.linear_speed
            twist = (direction * speed, 0.0)

            def progress():
                phi_l, phi_r, _ = self._read_integrators()
                return abs(self.wheel_radius * (phi_l + phi_r) / 2.0)
        else:
            target, speed = abs(self.spin_angle), self.angular_speed
            twist = (0.0, direction * speed)

            def progress():
                phi_l, phi_r, gyro = self._read_integrators()
                if self.use_imu:
                    return abs(gyro)
                return abs(self.wheel_radius * (phi_r - phi_l) / (self.track_width * self.chi))

        # Keep commanding until the (gyro / nominal wheel) progress reaches the target. Any
        # mismatch is fine: the reference measures where the rover actually ended up.
        timeout = 3.0 * target / speed + 5.0
        started = self.get_clock().now()
        while progress() < target:
            if (self.get_clock().now() - started).nanoseconds * 1e-9 > timeout:
                self.stop_robot()
                raise CalibrationAborted(
                    f'{kind} run did not reach its target within {timeout:.0f} s (stuck?)')
            self._publish(*twist)
            time.sleep(0.05)

        self._settle()  # integrators keep running so the coast-down is included
        end = self._capture()
        phi_l, phi_r, gyro = self._read_integrators()

        if kind == STRAIGHT:
            guess = (self.wheel_radius * (phi_l + phi_r) / 2.0, 0.0,
                     gyro if self.use_imu else 0.0)
        else:
            guess = (0.0, 0.0, gyro if self.use_imu else
                     self.wheel_radius * (phi_r - phi_l) / (self.track_width * self.chi))
        x, y, yaw, quality = self._measure(start, end, guess)
        segment = Segment(kind, direction, phi_l, phi_r, x, y, yaw, gyro, **quality)

        if quality:
            problems = []
            if segment.inlier_ratio < self.min_inlier_ratio:
                problems.append(f'inlier ratio {segment.inlier_ratio:.2f}')
            if kind == STRAIGHT and segment.observability < self.min_observability:
                problems.append(f'translation poorly constrained ({segment.observability:.3f})')
            if (kind == SPIN and math.isfinite(gyro) and
                    abs(yaw - gyro) > self.max_gyro_disagreement):
                problems.append(f'{math.degrees(yaw - gyro):+.1f} deg off the gyro')
            if problems:
                segment.valid = False
                segment.note = 'scan match rejected: ' + ', '.join(problems)
        return segment

    def _log_segment(self, n, total, s):
        line = (f'[{n}/{total}] {s.kind} {"+" if s.direction > 0 else "-"}: '
                f'ref d={s.ref_distance:+.4f} m yaw={math.degrees(s.ref_yaw):+.2f} deg, '
                f'wheels L={s.phi_left:+.3f} R={s.phi_right:+.3f} rad')
        if math.isfinite(s.gyro_yaw):
            line += f', gyro={math.degrees(s.gyro_yaw):+.2f} deg'
        if math.isfinite(s.rmse):
            line += f', icp rmse={s.rmse * 1000:.1f} mm inliers={s.inlier_ratio:.2f}'
        if s.valid:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(f'{line} -- discarded, {s.note}')

    def run(self):
        self.get_logger().info('Waiting for /joint_states' +
                               (', /scan' if self.reference == 'scan' or self.check_clearance else '') +
                               (' and /imu' if self.use_imu else ''))
        self._wait_for(lambda: self._last_wheels is not None, 15.0, '/joint_states')
        if self.reference == 'scan' or self.check_clearance:
            self._wait_for(lambda: self._scan is not None, 15.0, '/scan')
            frame = self._scan.header.frame_id
            self.base_to_laser = self._lookup_se2(self.base_frame, frame)
        if self.use_imu:
            self._wait_for(lambda: self._imu_seen, 15.0, '/imu (set use_imu:=false to run without)')
        if self.reference == 'tf':
            self._lookup_se2(self.reference_frame, self.base_frame)
        self._read_kinematics()

        plan = ([(STRAIGHT, 1 if i % 2 == 0 else -1) for i in range(self.straight_runs)] +
                [(SPIN, 1 if i % 2 == 0 else -1) for i in range(self.spin_runs)])
        self.get_logger().warning(
            f'Starting in {self.start_delay:.0f} s: {self.straight_runs} straight runs of '
            f'{self.straight_distance:.2f} m (forward/back) and {self.spin_runs} spins of '
            f'{math.degrees(self.spin_angle):.0f} deg. Keep clear of the rover; Ctrl-C stops it.')
        self._sleep(self.start_delay)

        segments = []
        for n, (kind, direction) in enumerate(plan, start=1):
            segment = self._run_segment(kind, direction)
            segments.append(segment)
            self._log_segment(n, len(plan), segment)
        self.stop_robot()
        self._report(segments)

    def _report(self, segments):
        log = self.get_logger()
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(self.output_dir, exist_ok=True)
        csv_path = os.path.join(self.output_dir, f'odom_calibration_{stamp}.csv')
        fields = list(Segment.__dataclass_fields__)
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for s in segments:
                writer.writerow({k: getattr(s, k) for k in fields})
        log.info(f'Segments written to {csv_path}')

        radius = fitting.fit_wheel_radius(segments)
        wheel_radius = radius.value if radius else self.wheel_radius
        if radius:
            log.info(f'wheel_radius: {self.wheel_radius:.5f} -> {radius.value:.5f} m '
                     f'({100.0 * (radius.value / self.wheel_radius - 1.0):+.2f} %, '
                     f'per-run spread {radius.spread * 1000:.2f} mm)')
        else:
            log.error('No valid straight run: wheel_radius not estimated, chi uses the current one')
        chi = fitting.fit_chi(segments, wheel_radius, self.track_width)
        if chi:
            log.info(f'chi: {self.chi:.4f} -> {chi.value:.4f} '
                     f'(effective track {self.track_width * chi.value:.4f} m, '
                     f'per-run spread {chi.spread:.4f})')
        else:
            log.error('No valid spin: chi not estimated')
        gyro_scale = fitting.fit_gyro_scale(segments)
        if gyro_scale is not None:
            log.info(f'IMU gyro scale vs lidar: {gyro_scale:.4f} (1.0 = exact)')
        drift = fitting.straight_heading_drift(segments)
        if drift is not None:
            log.info(f'Heading drift on straight runs: {math.degrees(drift):+.2f} deg/m '
                     '(left/right wheel imbalance)')
        if not radius and not chi:
            raise CalibrationAborted('no usable runs, nothing to write')

        # track_width too: chi is only meaningful together with the track_width it was fitted for
        values = {'wheel_radius': round(wheel_radius, 6),
                  'track_width': round(self.track_width, 6),
                  'chi': round(chi.value if chi else self.chi, 6)}
        yaml_path = os.path.join(self.output_dir, f'odom_calibration_{stamp}.yaml')
        with open(yaml_path, 'w') as f:
            f.write(f'# kvn_odom_calibration {stamp}: copy into kvn_robot_bringup/config/'
                    'controllers.yaml\n')
            yaml.safe_dump({name.lstrip('/').split('/')[-1]: {'ros__parameters': dict(values)}
                            for name in (self.controller_node, self.odometry_node)},
                           f, sort_keys=False)
        log.info(f'Parameters written to {yaml_path}')

        if self.apply_results:
            params = [Parameter(k, value=float(v)) for k, v in values.items()]
            for client, name in ((self.controller_client, self.controller_node),
                                 (self.odometry_client, self.odometry_node)):
                if client is not None:
                    self._set_params(client, params, f'apply to {name}')
                    log.info(f'Applied {values} to {name} (runtime only, not saved)')


def main(args=None):
    # Own SIGINT handling: on Ctrl-C the rover must still be stopped and parameters restored
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OdomCalibration()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 0
    try:
        node.run()
    except CalibrationAborted as e:
        node.get_logger().error(f'Calibration aborted: {e}')
        code = 1
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted, stopping the rover')
        code = 130
    finally:
        node.stop_robot()
        node.restore_parameters()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
