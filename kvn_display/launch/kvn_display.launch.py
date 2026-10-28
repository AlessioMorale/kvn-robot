from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_display_arg = DeclareLaunchArgument(
        "use_display",
        default_value="true",
        description="Start the status display node",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_display"), "config", "kvn_display.yaml"]
        ),
        description="Parameters file for kvn_display_node",
    )

    display_node = Node(
        package="kvn_display",
        executable="kvn_display_node",
        name="kvn_display_node",
        parameters=[LaunchConfiguration("params_file")],
        condition=IfCondition(LaunchConfiguration("use_display")),
    )
    return LaunchDescription([use_display_arg, params_file_arg, display_node])
