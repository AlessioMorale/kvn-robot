# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""ROS 2 TF bridge from ORB-SLAM3 camera pose to map -> odom -> base_link.

kvn_ekf (fuse) owns odom -> base_link. ORB-SLAM3 publishes the tracked sensor
pose (default /orbslam3/camera_pose) in the map frame. This node computes
T_map_odom = T_map_base * inv(T_odom_base) and broadcasts map -> odom, keeping
a valid single-parent TF tree.
"""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import tf2_ros

from kvn_orbslam3.geometry import Quaternion, Transform3D, compute_map_to_odom


def _transform_from_pose(msg: PoseStamped) -> Transform3D:
    p = msg.pose.position
    q = msg.pose.orientation
    return Transform3D(
        (float(p.x), float(p.y), float(p.z)),
        Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)).normalized(),
    )


def _transform_from_tf(msg: TransformStamped) -> Transform3D:
    t = msg.transform.translation
    q = msg.transform.rotation
    return Transform3D(
        (float(t.x), float(t.y), float(t.z)),
        Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)).normalized(),
    )


class OrbSlam3TfBridgeNode(Node):
    """Bridges ORB-SLAM3 sensor poses to the ROS 2 map -> odom TF tree."""

    def __init__(self) -> None:
        super().__init__('orbslam3_tf_bridge')

        self.pose_topic = self.declare_parameter('pose_topic', '/orbslam3/camera_pose').value
        self.base_pose_topic = self.declare_parameter('base_pose_topic', '/orbslam3/base_pose').value
        self.status_topic = self.declare_parameter('status_topic', '/orbslam3/tracking_status').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.sensor_frame = self.declare_parameter(
            'sensor_frame', 'oak_rgb_camera_optical_frame'
        ).value
        self.project_to_2d = bool(self.declare_parameter('project_to_2d', True).value)
        self.pose_timeout_s = float(self.declare_parameter('pose_timeout_s', 1.5).value)
        pub_period = float(self.declare_parameter('transform_publish_period', 0.05).value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self._base_pose_pub = self.create_publisher(PoseStamped, self.base_pose_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self._last_map_to_odom: Transform3D | None = None
        self._last_pose_time = None
        self._last_status: str = ''

        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_timer(pub_period, self._on_timer)
        self.create_timer(1.0, self._on_diagnostics)
        self._publish_status('waiting')

    def _publish_status(self, state: str) -> None:
        if state == self._last_status:
            return
        self._last_status = state
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)

    def _on_pose(self, msg: PoseStamped) -> None:
        try:
            t_map_sensor = _transform_from_pose(msg)
        except ValueError as exc:
            self.get_logger().warning(f'Invalid ORB-SLAM3 pose quaternion: {exc}', throttle_duration_sec=5.0)
            return
        if not t_map_sensor.is_finite():
            return

        sensor_frame = msg.header.frame_id or self.sensor_frame
        try:
            tf_base_sensor = self._tf_buffer.lookup_transform(
                self.base_frame, sensor_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            tf_odom_base = self._tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().debug(f'Waiting for TF ({self.odom_frame}->{self.base_frame}->{sensor_frame}): {exc}')
            return

        t_map_odom, t_map_base_3d = compute_map_to_odom(
            t_map_sensor,
            _transform_from_tf(tf_base_sensor),
            _transform_from_tf(tf_odom_base),
            project_to_2d=self.project_to_2d,
        )
        self._last_map_to_odom = t_map_odom
        self._last_pose_time = self.get_clock().now()
        self._publish_status('tracking')

        stamp = msg.header.stamp if (msg.header.stamp.sec or msg.header.stamp.nanosec) else self._last_pose_time.to_msg()
        self._broadcast_map_to_odom(t_map_odom, stamp)

        base_pose = PoseStamped()
        base_pose.header.stamp = stamp
        base_pose.header.frame_id = self.map_frame
        base_pose.pose.position.x = t_map_base_3d.translation[0]
        base_pose.pose.position.y = t_map_base_3d.translation[1]
        base_pose.pose.position.z = t_map_base_3d.translation[2]
        base_pose.pose.orientation.x = t_map_base_3d.rotation.x
        base_pose.pose.orientation.y = t_map_base_3d.rotation.y
        base_pose.pose.orientation.z = t_map_base_3d.rotation.z
        base_pose.pose.orientation.w = t_map_base_3d.rotation.w
        self._base_pose_pub.publish(base_pose)

    def _broadcast_map_to_odom(self, tf_val: Transform3D, stamp) -> None:
        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = tf_val.translation[0]
        out.transform.translation.y = tf_val.translation[1]
        out.transform.translation.z = tf_val.translation[2]
        out.transform.rotation.x = tf_val.rotation.x
        out.transform.rotation.y = tf_val.rotation.y
        out.transform.rotation.z = tf_val.rotation.z
        out.transform.rotation.w = tf_val.rotation.w
        self._tf_broadcaster.sendTransform(out)

    def _on_timer(self) -> None:
        if self._last_map_to_odom is None or self._last_pose_time is None:
            return
        now = self.get_clock().now()
        age_s = (now - self._last_pose_time).nanoseconds * 1e-9
        if age_s > self.pose_timeout_s:
            self._publish_status('lost')
            return
        # Republish last valid transform to bridge brief ORB-SLAM3 Local BA pauses
        self._broadcast_map_to_odom(self._last_map_to_odom, now.to_msg())

    def _on_diagnostics(self) -> None:
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'orbslam3_tf_bridge'
        status.hardware_id = 'oak_d_lite_ism330'
        if self._last_status == 'tracking':
            status.level = DiagnosticStatus.OK
            status.message = 'Tracking active'
        elif self._last_status == 'waiting':
            status.level = DiagnosticStatus.WARN
            status.message = 'Waiting for ORB-SLAM3 pose'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'ORB-SLAM3 tracking lost'
        status.values = [KeyValue(key='state', value=self._last_status or 'waiting')]
        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbSlam3TfBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
