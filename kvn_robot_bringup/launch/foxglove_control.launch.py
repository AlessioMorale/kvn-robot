"""
foxglove_bridge that may also COMMAND the rover: teleop and navigation goals from Foxglove/Lichtblick.

Opt-in exception to rule R1 (WiFi never commands the robot), as its own systemd unit
(kvn-foxglove-control.service) instead of the read-only kvn-foxglove.service. Same bridge, same
limits, same viewable topics (foxglove_bridge.launch.py), plus the clientPublish capability, meant
for these topics:

  /cmd_vel          geometry_msgs/Twist        Teleop panel. skid_steer_controller stops the rover
                                               cmd_vel_timeout (1 s) after the last message, e.g.
                                               when the WiFi drops.
  /goal_pose        geometry_msgs/PoseStamped  3D panel "Publish pose", frame map: Nav2 navigates
                                               there (bt_navigator's goal_pose subscription).
  /explore/resume   std_msgs/Bool              false pauses exploration, true resumes it.

WARNING: foxglove_bridge 3.6.2 (lyrical) does NOT enforce client_topic_whitelist. The parameter is
set, but a client can still advertise and publish ANY topic (checked: /forbidden got through). The
list below documents the intent and takes effect once the bridge enforces it. Until then the only
protection is who can reach TCP 8765: nftables (ZeroTier only) and the ZeroTier flow rules.

No services, parameters or actions are exposed, so a running Nav2 goal cannot be cancelled from
here: send a new goal, or switch mode (kvn-drive.target stops Nav2).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_TOPICS = [
    "^/cmd_vel$",
    "^/goal_pose$",
    "^/explore/resume$",
]


def generate_launch_description():
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kvn_robot_bringup"), "launch", "foxglove_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "capabilities": str(["clientPublish"]),
            "client_topic_whitelist": str(CONTROL_TOPICS),
        }.items(),
    )
    return LaunchDescription([bridge])
