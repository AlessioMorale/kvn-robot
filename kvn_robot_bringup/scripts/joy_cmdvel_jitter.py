#!/usr/bin/env python3
"""
Measures /joy -> /cmd_vel period jitter and latency with receive timestamps (plan T4.3).

Run while the bridge streams and video runs at maximum settings. Budget (T0.4):
p99 period jitter <= 5 ms.

  ros2 run kvn_robot_bringup joy_cmdvel_jitter.py --duration 60
"""

import argparse
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]


def summarize(name, periods_ms, nominal_ms=None):
    if len(periods_ms) < 2:
        print(f"{name}: not enough samples ({len(periods_ms)})")
        return float("nan")
    ref = nominal_ms if nominal_ms else statistics.median(periods_ms)
    dev = [abs(p - ref) for p in periods_ms]
    print(
        f"{name}: n={len(periods_ms) + 1} period p50={pct(periods_ms, 50):.2f} ms "
        f"p99={pct(periods_ms, 99):.2f} ms max={max(periods_ms):.2f} ms | "
        f"jitter |period-ref| p50={pct(dev, 50):.2f} ms p99={pct(dev, 99):.2f} ms "
        f"(ref {ref:.2f} ms)"
    )
    return pct(dev, 99)


class Jitter(Node):
    def __init__(self, joy_topic, cmd_topic, stamped):
        super().__init__("joy_cmdvel_jitter")
        self.joy_t, self.cmd_t, self.latency = [], [], []
        self.last_joy = None
        self.create_subscription(Joy, joy_topic, self.on_joy, qos_profile_sensor_data)
        msg_type = TwistStamped if stamped else Twist
        self.create_subscription(msg_type, cmd_topic, self.on_cmd, 10)

    def on_joy(self, _msg):
        self.joy_t.append(time.monotonic())
        self.last_joy = self.joy_t[-1]

    def on_cmd(self, _msg):
        now = time.monotonic()
        self.cmd_t.append(now)
        if self.last_joy is not None:
            self.latency.append((now - self.last_joy) * 1000.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--joy-topic", default="/joy")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--stamped", action="store_true", help="/cmd_vel is TwistStamped")
    ap.add_argument("--budget-ms", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = Jitter(args.joy_topic, args.cmd_topic, args.stamped)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    diffs = lambda t: [(b - a) * 1000.0 for a, b in zip(t, t[1:])]  # noqa: E731
    summarize(args.joy_topic, diffs(node.joy_t))
    p99 = summarize(args.cmd_topic, diffs(node.cmd_t))
    if node.latency:
        print(
            f"/joy -> /cmd_vel lag (last joy before each cmd): p50={pct(node.latency, 50):.2f} "
            f"ms p99={pct(node.latency, 99):.2f} ms"
        )
    ok = p99 == p99 and p99 <= args.budget_ms
    print(
        f'RESULT: {"PASS" if ok else "FAIL"} (cmd_vel p99 jitter {p99:.2f} ms, '
        f"budget {args.budget_ms} ms)"
    )
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
