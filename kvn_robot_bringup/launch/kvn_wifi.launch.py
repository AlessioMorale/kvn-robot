"""
Opt-in WiFi enhancement: read-only foxglove_bridge + on-demand video streamer.

Not part of the control stack. Run it as its own (low priority) process, see
systemd/kvn-wifi.service. Arguments of the two included launch files are forwarded.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kvn_robot_bringup"), "launch", "kvn_foxglove_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "address": LaunchConfiguration("address"),
            "send_buffer_limit": LaunchConfiguration("send_buffer_limit"),
        }.items(),
    )
    video = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kvn_video_streamer"), "launch", "video_streamer.launch.py"]
            )
        ),
        launch_arguments={
            "source_pipeline": LaunchConfiguration("source_pipeline"),
            "webrtc": LaunchConfiguration("webrtc"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("address", default_value="0.0.0.0"),
            DeclareLaunchArgument("send_buffer_limit", default_value="4000000"),
            DeclareLaunchArgument("source_pipeline", default_value="v4l2src device=/dev/video0"),
            DeclareLaunchArgument("webrtc", default_value="false"),
            bridge,
            video,
        ]
    )
