#!/usr/bin/env python3
# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from explore_lite_msgs.msg import ExploreStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import String


MODE_SETTINGS = {
    "starting_up": ((0, 0, 255), 2),
    "standby": ((255, 255, 255), 0),
    "teleop": ((0, 255, 0), 0),
    "assisted_teleop": ((160, 0, 255), 0),
    "navigation": ((0, 64, 255), 0),
    "exploration": ((0, 255, 180), 0),
    "returning_home": ((255, 255, 0), 1),
    "battery_low": ((255, 96, 0), 1),
    "battery_critical": ((255, 0, 0), 1),
    "warning": ((255, 160, 0), 0),
    "fault": ((255, 0, 255), 1),
}


def select_mode(
    robot_status: str | None,
    battery_voltage: float | None,
    joy_enabled: bool,
    exploring: bool,
    navigating: bool,
    low_battery_voltage: float,
    critical_battery_voltage: float,
) -> str:
    if battery_voltage is not None:
        if (
            not math.isfinite(battery_voltage)
            or battery_voltage <= critical_battery_voltage
        ):
            return "battery_critical"

    state, _, code = (robot_status or "").partition(":")
    if state == "FLT":
        return "battery_critical" if code.startswith("BATT") else "fault"
    if battery_voltage is not None and battery_voltage <= low_battery_voltage:
        return "battery_low"
    if state == "WRN":
        if code.startswith("BATT"):
            return "battery_low"
        return "warning"

    if robot_status is None:
        return "starting_up"
    if exploring:
        return "exploration"
    if navigating:
        return "navigation"
    if joy_enabled:
        return "teleop"
    return "standby"


def resolve_mode(signal: str | None, fallback_mode: str) -> str:
    """The orchestrator's signal when it is fresh and known, else the node's own choice.

    The signalling tree in kvn_orchestrator decides the light while it runs; it only runs with
    Nav2, so without it the light still follows battery, faults and teleop locally.
    """
    if signal in MODE_SETTINGS:
        return signal
    return fallback_mode


class WledSignalNode(Node):
    def __init__(self) -> None:
        super().__init__("kvn_wled_signal_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("segment_id", 0)
        self.declare_parameter("brightness", 64)
        self.declare_parameter("mode_topic", "/robot_mode")
        self.declare_parameter("signal_topic", "/orchestrator/signal")
        self.declare_parameter("signal_timeout_s", 3.0)
        self.declare_parameter("robot_status_topic", "/robot_status")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("explore_status_topic", "/explore/status")
        self.declare_parameter(
            "navigation_status_topics",
            ["/navigate_to_pose/_action/status", "/navigate_through_poses/_action/status"],
        )
        self.declare_parameter("enable_button", 0)
        self.declare_parameter("low_battery_voltage", 13.6)
        self.declare_parameter("critical_battery_voltage", 12.4)
        self.declare_parameter("joy_timeout_s", 1.0)
        self.declare_parameter("battery_timeout_s", 3.0)
        self.declare_parameter("robot_status_timeout_s", 3.0)
        self.declare_parameter("navigation_status_timeout_s", 2.0)
        self.declare_parameter("update_period_s", 0.5)

        serial_port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value
        self._segment_id = self.get_parameter("segment_id").value
        self._brightness = self.get_parameter("brightness").value
        self._low_battery_voltage = self.get_parameter("low_battery_voltage").value
        self._critical_battery_voltage = self.get_parameter(
            "critical_battery_voltage"
        ).value
        self._joy_timeout_s = self.get_parameter("joy_timeout_s").value
        self._battery_timeout_s = self.get_parameter("battery_timeout_s").value
        self._robot_status_timeout_s = self.get_parameter(
            "robot_status_timeout_s"
        ).value
        self._navigation_status_timeout_s = self.get_parameter(
            "navigation_status_timeout_s"
        ).value
        self._signal_timeout_s = self.get_parameter("signal_timeout_s").value
        update_period_s = self.get_parameter("update_period_s").value
        self._enable_button = self.get_parameter("enable_button").value
        navigation_topics = self.get_parameter("navigation_status_topics").value

        if not serial_port or baudrate <= 0:
            raise ValueError("serial_port must be set and baudrate must be > 0")
        if self._segment_id < 0 or not 1 <= self._brightness <= 255:
            raise ValueError("segment_id must be >= 0 and brightness must be in [1, 255]")
        if self._enable_button < 0:
            raise ValueError("enable_button must be >= 0")
        if not (
            math.isfinite(self._critical_battery_voltage)
            and math.isfinite(self._low_battery_voltage)
            and 0 < self._critical_battery_voltage < self._low_battery_voltage
        ):
            raise ValueError("battery thresholds must be finite, > 0, and critical < low")
        timeouts = (
            self._joy_timeout_s,
            self._battery_timeout_s,
            self._robot_status_timeout_s,
            self._navigation_status_timeout_s,
            self._signal_timeout_s,
            update_period_s,
        )
        if any(not math.isfinite(value) or value <= 0 for value in timeouts):
            raise ValueError("timeouts and update_period_s must be finite and > 0")
        if not isinstance(navigation_topics, list) or any(
            not topic for topic in navigation_topics
        ):
            raise ValueError("navigation_status_topics must not contain empty topic names")

        try:
            from serial import SerialException
            from wled_adapter.adapter import Adapter
            from wled_adapter.connection import SerialConnection
            from wled_adapter.state import Seg, State
        except ImportError as exc:
            raise RuntimeError(
                "WLED signalling requires wled-adapter; install it for the ROS runtime user "
                "with `python3 -m pip install --user "
                "'git+https://github.com/AlessioMorale/wled-adapter.git'`"
            ) from exc

        self._serial_exception = SerialException
        self._seg_type = Seg
        self._state_type = State
        self._adapter = Adapter(SerialConnection(serial_port, baudrate))

        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_publisher = self.create_publisher(
            String, self.get_parameter("mode_topic").value, mode_qos
        )
        self._signal: str | None = None
        self._signal_time: float | None = None
        self._status: str | None = None
        self._status_time: float | None = None
        self._battery_voltage: float | None = None
        self._battery_time: float | None = None
        self._joy_enabled = False
        self._joy_time: float | None = None
        self._exploring = False
        self._navigation_status: dict[str, bool] = {
            topic: False for topic in navigation_topics
        }
        self._navigation_times: dict[str, float | None] = {
            topic: None for topic in navigation_topics
        }
        self._last_mode: str | None = None
        self._last_written_mode: str | None = None
        self._subscriptions = [
            self.create_subscription(
                String,
                self.get_parameter("robot_status_topic").value,
                self._on_robot_status,
                10,
            ),
            self.create_subscription(
                String,
                self.get_parameter("signal_topic").value,
                self._on_signal,
                mode_qos,
            ),
            self.create_subscription(
                BatteryState,
                self.get_parameter("battery_topic").value,
                self._on_battery,
                10,
            ),
            self.create_subscription(
                Joy,
                self.get_parameter("joy_topic").value,
                self._on_joy,
                10,
            ),
            self.create_subscription(
                ExploreStatus,
                self.get_parameter("explore_status_topic").value,
                self._on_explore_status,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            ),
        ]
        for topic in navigation_topics:
            self._subscriptions.append(
                self.create_subscription(
                    GoalStatusArray,
                    topic,
                    lambda msg, action_topic=topic: self._on_navigation_status(
                        action_topic, msg
                    ),
                    10,
                )
            )

        self._timer = self.create_timer(update_period_s, self._evaluate)
        self._evaluate()
        self.get_logger().info(
            f"Controlling WLED segment {self._segment_id} via {serial_port}"
        )

    def _on_signal(self, msg: String) -> None:
        self._signal = msg.data.strip()
        self._signal_time = time.monotonic()
        if self._signal not in MODE_SETTINGS:
            self.get_logger().warning(
                f"Unknown orchestrator signal '{self._signal}', using the local mode"
            )
        self._evaluate()

    def _on_robot_status(self, msg: String) -> None:
        self._status = msg.data.strip()
        self._status_time = time.monotonic()
        self._evaluate()

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_voltage = msg.voltage
        self._battery_time = time.monotonic()
        self._evaluate()

    def _on_joy(self, msg: Joy) -> None:
        self._joy_enabled = (
            self._enable_button < len(msg.buttons)
            and msg.buttons[self._enable_button] != 0
        )
        self._joy_time = time.monotonic()
        self._evaluate()

    def _on_explore_status(self, msg: ExploreStatus) -> None:
        self._exploring = msg.status in (
            ExploreStatus.EXPLORATION_STARTED,
            ExploreStatus.EXPLORATION_IN_PROGRESS,
        )
        self._evaluate()

    def _on_navigation_status(self, topic: str, msg: GoalStatusArray) -> None:
        active_statuses = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self._navigation_status[topic] = any(
            goal.status in active_statuses for goal in msg.status_list
        )
        self._navigation_times[topic] = time.monotonic()
        self._evaluate()

    def _evaluate(self) -> None:
        now = time.monotonic()
        status = self._status
        if (
            self._status_time is not None
            and now - self._status_time > self._robot_status_timeout_s
        ):
            status = "FLT:STATUS"

        battery_voltage = self._battery_voltage
        if (
            self._battery_time is not None
            and now - self._battery_time > self._battery_timeout_s
        ):
            battery_voltage = None
        joy_enabled = (
            self._joy_enabled
            and self._joy_time is not None
            and now - self._joy_time <= self._joy_timeout_s
        )
        navigating = any(
            active
            and self._navigation_times[topic] is not None
            and now - self._navigation_times[topic]
            <= self._navigation_status_timeout_s
            for topic, active in self._navigation_status.items()
        )

        signal = (
            self._signal
            if self._signal_time is not None
            and now - self._signal_time <= self._signal_timeout_s
            else None
        )
        local_mode = select_mode(
            status,
            battery_voltage,
            joy_enabled,
            self._exploring,
            navigating,
            self._low_battery_voltage,
            self._critical_battery_voltage,
        )
        mode = resolve_mode(signal, local_mode)
        if mode != self._last_mode:
            self._last_mode = mode
            message = String()
            message.data = mode
            self._mode_publisher.publish(message)
            self.get_logger().info(f"Robot mode: {mode}")

        if mode == self._last_written_mode:
            return
        color, effect = MODE_SETTINGS[mode]
        try:
            self._adapter.send_state(
                self._state_type(
                    seg=[
                        self._seg_type(
                            id=self._segment_id,
                            on=True,
                            bri=self._brightness,
                            fx=effect,
                            col=[list(color), [0, 0, 0], [0, 0, 0]],
                        )
                    ]
                )
            )
        except (OSError, self._serial_exception) as exc:
            self.get_logger().error(f"Failed to update WLED mode {mode}: {exc}")
            return
        self._last_written_mode = mode

    def destroy_node(self) -> bool:
        self._adapter.cleanup()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = WledSignalNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
