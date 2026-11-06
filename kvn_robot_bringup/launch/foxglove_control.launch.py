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
                                               there (bt_navigator's goal_pose subscription). The
                                               rover only moves with the AUX5 mode switch on
                                               autonomous (the orchestrator's autonomy permit).

NOTE: /cmd_vel from Foxglove bypasses teleop_twist_joy_node (kvn_joy_teleop), which arbitrates
between the RC sticks and Nav2: the handset cannot override a Foxglove teleop command, and both
publish to the controller while either is active.
  /explore/resume   std_msgs/Bool              false pauses exploration, true resumes it.
  /calibration/command  std_msgs/String        START / NEXT / SKIP / ABORT / SAVE for the
                                               calibration supervisor (kvn-calibration.target);
                                               its status is on /calibration/status.

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
    "^/calibration/command$",
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
