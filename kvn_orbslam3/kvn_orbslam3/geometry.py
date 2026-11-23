# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Pure-Python SE(3)/SE(2) transform utilities for the ORB-SLAM3 TF bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Quaternion:
    """Unit quaternion in ROS (x, y, z, w) order."""

    x: float
    y: float
    z: float
    w: float

    @classmethod
    def identity(cls) -> Quaternion:
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_yaw(cls, yaw: float) -> Quaternion:
        half = 0.5 * yaw
        return cls(0.0, 0.0, math.sin(half), math.cos(half))

    @classmethod
    def from_rpy(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
        cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
        cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
        return cls(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        ).normalized()

    def norm(self) -> float:
        return math.sqrt(
            self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
        )

    def normalized(self) -> Quaternion:
        n = self.norm()
        if not math.isfinite(n) or n <= 1e-12:
            raise ValueError('Quaternion norm must be finite and > 0')
        inv = 1.0 / n
        return Quaternion(self.x * inv, self.y * inv, self.z * inv, self.w * inv)

    def conjugate(self) -> Quaternion:
        q = self.normalized()
        return Quaternion(-q.x, -q.y, -q.z, q.w)

    def __mul__(self, other: Quaternion) -> Quaternion:
        return Quaternion(
            x=self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y=self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z=self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w=self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        ).normalized()

    def rotate_vector(self, v: tuple[float, float, float]) -> tuple[float, float, float]:
        q = self.normalized()
        vx, vy, vz = v
        # v' = v + 2 * q_xyz x (q_xyz x v + w * v)
        tx = 2.0 * (q.y * vz - q.z * vy)
        ty = 2.0 * (q.z * vx - q.x * vz)
        tz = 2.0 * (q.x * vy - q.y * vx)
        return (
            vx + q.w * tx + (q.y * tz - q.z * ty),
            vy + q.w * ty + (q.z * tx - q.x * tz),
            vz + q.w * tz + (q.x * ty - q.y * tx),
        )

    def yaw(self) -> float:
        q = self.normalized()
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Transform3D:
    """Rigid 3D transform T_parent_child: p_parent = R * p_child + t."""

    translation: tuple[float, float, float]
    rotation: Quaternion

    @classmethod
    def identity(cls) -> Transform3D:
        return cls((0.0, 0.0, 0.0), Quaternion.identity())

    def is_finite(self) -> bool:
        return all(
            math.isfinite(v)
            for v in (
                *self.translation,
                self.rotation.x,
                self.rotation.y,
                self.rotation.z,
                self.rotation.w,
            )
        )

    def inverse(self) -> Transform3D:
        r_inv = self.rotation.conjugate()
        tx, ty, tz = r_inv.rotate_vector(self.translation)
        return Transform3D((-tx, -ty, -tz), r_inv)

    def compose(self, other: Transform3D) -> Transform3D:
        """Return self * other (T_a_b.compose(T_b_c) == T_a_c)."""
        rx, ry, rz = self.rotation.rotate_vector(other.translation)
        tx, ty, tz = self.translation
        return Transform3D(
            (tx + rx, ty + ry, tz + rz),
            self.rotation * other.rotation,
        )

    def project_to_2d(self) -> Transform3D:
        """Flatten transform to SE(2): keep (x, y, yaw) and zero (z, roll, pitch)."""
        return Transform3D(
            (self.translation[0], self.translation[1], 0.0),
            Quaternion.from_yaw(self.rotation.yaw()),
        )


def compute_map_to_base(
    t_map_sensor: Transform3D,
    t_base_sensor: Transform3D,
) -> Transform3D:
    """Convert an ORB-SLAM3 sensor pose in map frame to T_map_base_link.

    T_map_base = T_map_sensor * inv(T_base_sensor)
    """
    return t_map_sensor.compose(t_base_sensor.inverse())


def compute_map_to_odom(
    t_map_sensor: Transform3D,
    t_base_sensor: Transform3D,
    t_odom_base: Transform3D,
    project_to_2d: bool = True,
) -> tuple[Transform3D, Transform3D]:
    """Compute (T_map_odom, T_map_base_3d) for ROS 2 TF tree integration.

    kvn_ekf (fuse) owns odom -> base_link. ORB-SLAM3 supplies T_map_sensor.
    Publishing T_map_odom = T_map_base * inv(T_odom_base) preserves the single-parent
    TF tree: map -> odom -> base_link.

    When project_to_2d is True (default for 2D Nav2 costmaps on a ground rover),
    T_map_odom is flattened to SE(2) so VIO roll/pitch/z jitter does not tilt /map.
    """
    t_map_base_3d = compute_map_to_base(t_map_sensor, t_base_sensor)
    t_map_base_for_tf = t_map_base_3d.project_to_2d() if project_to_2d else t_map_base_3d
    t_odom_base_for_tf = t_odom_base.project_to_2d() if project_to_2d else t_odom_base
    t_map_odom = t_map_base_for_tf.compose(t_odom_base_for_tf.inverse())
    if project_to_2d:
        t_map_odom = t_map_odom.project_to_2d()
    return t_map_odom, t_map_base_3d
