from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_video_streamer"), "config", "video_streamer.yaml"]
        ),
        description="Parameter file for the video streamer",
    )
    source_arg = DeclareLaunchArgument(
        "source_pipeline",
        default_value="v4l2src device=/dev/video0",
        description='GStreamer source fragment (e.g. "videotestsrc is-live=true")',
    )

    webrtc_arg = DeclareLaunchArgument(
        "webrtc",
        default_value="false",
        description="Also stream over WebRTC (needs gst-plugins-rs webrtcsink and a signalling server)",
    )

    video_streamer = Node(
        package="kvn_video_streamer",
        executable="video_streamer_node",
        name="video_streamer",
        parameters=[
            LaunchConfiguration("config"),
            {
                "source_pipeline": LaunchConfiguration("source_pipeline"),
                "webrtc.enabled": ParameterValue(LaunchConfiguration("webrtc"), value_type=bool),
            },
        ],
        output="screen",
    )
    return LaunchDescription([config_arg, source_arg, webrtc_arg, video_streamer])
