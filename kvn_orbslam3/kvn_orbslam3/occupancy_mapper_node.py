# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Lightweight 2D occupancy grid mapper for ORB-SLAM3 + 2D LiDAR.

Publishes /map and /map_metadata (TRANSIENT_LOCAL) from /scan posed via
map -> lidar_attachment_link TF so Nav2 static_layer, explore_lite and
map_saver_server work identically to slam_toolbox.
"""

from __future__ import annotations

import math

from nav_msgs.msg import MapMetaData, OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

from kvn_orbslam3.geometry import Quaternion
from kvn_orbslam3.occupancy_grid import GridConfig, OccupancyGrid2D


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class OrbSlam3OccupancyMapperNode(Node):
    """Integrates /scan into a bounded 2D OccupancyGrid using ORB-SLAM3 TF."""

    def __init__(self) -> None:
        super().__init__('orbslam3_occupancy_mapper')

        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.map_metadata_topic = self.declare_parameter(
            'map_metadata_topic', '/map_metadata'
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        cfg = GridConfig(
            resolution=float(self.declare_parameter('resolution', 0.05).value),
            width_m=float(self.declare_parameter('width_m', 20.0).value),
            height_m=float(self.declare_parameter('height_m', 20.0).value),
            origin_x=float(self.declare_parameter('origin_x', -10.0).value),
            origin_y=float(self.declare_parameter('origin_y', -10.0).value),
            min_range=float(self.declare_parameter('min_range', 0.15).value),
            max_range=float(self.declare_parameter('max_range', 8.0).value),
            scan_stride=int(self.declare_parameter('scan_stride', 2).value),
        )
        self._grid = OccupancyGrid2D(cfg)

        self.min_travel_distance = float(
            self.declare_parameter('min_travel_distance', 0.10).value
        )
        self.min_travel_heading = float(
            self.declare_parameter('min_travel_heading', 0.10).value
        )
        map_update_interval = float(
            self.declare_parameter('map_update_interval', 2.0).value
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self._meta_pub = self.create_publisher(
            MapMetaData, self.map_metadata_topic, map_qos
        )

        self._last_pose: tuple[float, float, float] | None = None
        self._dirty = False

        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_timer(map_update_interval, self._publish_map)

    def _on_scan(self, msg: LaserScan) -> None:
        scan_frame = msg.header.frame_id or 'lidar_attachment_link'
        try:
            tf_map_scan = self._tf_buffer.lookup_transform(
                self.map_frame,
                scan_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except tf2_ros.TransformException:
            try:
                tf_map_scan = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    scan_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.02),
                )
            except tf2_ros.TransformException:
                return

        t = tf_map_scan.transform.translation
        q = tf_map_scan.transform.rotation
        yaw = Quaternion(q.x, q.y, q.z, q.w).yaw()
        sx, sy = float(t.x), float(t.y)

        if self._last_pose is not None:
            lx, ly, lyaw = self._last_pose
            dist = math.hypot(sx - lx, sy - ly)
            dyaw = abs(_angle_diff(yaw, lyaw))
            if dist < self.min_travel_distance and dyaw < self.min_travel_heading:
                return

        integrated = self._grid.integrate_scan(
            sensor_x=sx,
            sensor_y=sy,
            sensor_yaw=yaw,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            ranges=msg.ranges,
        )
        if integrated > 0:
            self._last_pose = (sx, sy, yaw)
            self._dirty = True

    def _publish_map(self) -> None:
        if not self._dirty:
            return
        self._dirty = False

        now = self.get_clock().now().to_msg()
        cfg = self._grid.config

        meta = MapMetaData()
        meta.map_load_time = now
        meta.resolution = cfg.resolution
        meta.width = self._grid.width
        meta.height = self._grid.height
        meta.origin.position.x = cfg.origin_x
        meta.origin.position.y = cfg.origin_y
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0

        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = self.map_frame
        grid.info = meta
        grid.data = self._grid.to_ros_data()

        self._meta_pub.publish(meta)
        self._map_pub.publish(grid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbSlam3OccupancyMapperNode()
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
