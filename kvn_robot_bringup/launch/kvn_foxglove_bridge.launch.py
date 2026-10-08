"""
Read-only foxglove_bridge for the handheld's Lichtblick view (design §4, §8, rule R1).

WiFi must never command the robot, so the bridge only exposes viewing:
  * no clientPublish, services, parameters, parametersSubscribe, assets or connectionGraph
    capability; every client-facing whitelist except topics never matches;
  * topics limited to the Lichtblick layout (`topic_whitelist` launch arg);
  * bounded per-client send buffer so a slow WiFi link drops data instead of
    growing memory on the robot;
  * Foxglove remote access (cloud gateway) off: the bridge is only reachable
    through the ZeroTier VPN (nftables rule, see systemd/README.md).

Checked with remote_controller/crates/test_tools/scripts/bridge_check.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Topics used by the Lichtblick layout. ECMAScript regexes, matched against the full name.
LAYOUT_TOPICS = [
    "^/video/compressed$",  # foxglove_msgs/CompressedVideo (kvn_video_streamer)
    "^/robot_status$",  # std_msgs/String (kvn_status)
    "^/battery_state$",  # sensor_msgs/BatteryState (hwmon)
    "^/odom$",  # nav_msgs/Odometry
    "^/cmd_vel$",  # geometry_msgs/Twist (view only; publishing is disabled)
    "^/diagnostics_agg$",  # diagnostic_msgs/DiagnosticArray
    "^/diagnostics$",
]

# Never matches anything: the idiom foxglove_bridge itself uses for "nothing",
# because an empty list cannot be passed as a typed string-array parameter.
NOTHING = ["(?!)"]

# The bridge treats an empty capability list as "unset" in some versions, so it is
# given one capability string that grants nothing ("time" only applies with
# use_sim_time, which stays false).
READ_ONLY_CAPABILITIES = ["time"]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "address",
            default_value="0.0.0.0",
            description="Bind address. The nftables rule limits port 8765 to the ZeroTier "
            "interface; set the robot ZeroTier IP here to also bind narrowly.",
        ),
        DeclareLaunchArgument("port", default_value="8765", description="WebSocket port"),
        DeclareLaunchArgument(
            "topic_whitelist",
            default_value=str(LAYOUT_TOPICS),
            description="Topics visible to clients (list of ECMAScript regexes)",
        ),
        DeclareLaunchArgument(
            "send_buffer_limit",
            default_value="4000000",
            description="Per-client send buffer limit in bytes; older messages are dropped",
        ),
        DeclareLaunchArgument(
            "num_threads",
            default_value="1",
            description="ROS executor threads of the bridge (0 = one per core)",
        ),
        DeclareLaunchArgument(
            "max_qos_depth",
            default_value="5",
            description="Upper bound of the subscription queue depth",
        ),
    ]

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": LaunchConfiguration("address"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "tls": False,
                "capabilities": READ_ONLY_CAPABILITIES,
                "topic_whitelist": LaunchConfiguration("topic_whitelist"),
                "service_whitelist": NOTHING,
                "param_whitelist": NOTHING,
                "client_topic_whitelist": NOTHING,
                "asset_uri_allowlist": NOTHING,
                "send_buffer_limit": ParameterValue(
                    LaunchConfiguration("send_buffer_limit"), value_type=int
                ),
                "num_threads": ParameterValue(LaunchConfiguration("num_threads"), value_type=int),
                "max_qos_depth": ParameterValue(
                    LaunchConfiguration("max_qos_depth"), value_type=int
                ),
                "include_hidden": False,
                "use_sim_time": False,
                "remote_access": False,
                "sysinfo": False,
                "publish_client_count": False,
            }
        ],
    )

    return LaunchDescription(args + [foxglove_bridge])
