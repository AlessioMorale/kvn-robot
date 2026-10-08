from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_status_arg = DeclareLaunchArgument(
        "use_status",
        default_value="true",
        description="Start the robot status node publishing /robot_status",
    )
    status_config_arg = DeclareLaunchArgument(
        "status_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_status"), "config", "kvn_status.yaml"]
        ),
        description="Parameters file for kvn_status_node",
    )

    status_node = Node(
        package="kvn_status",
        executable="kvn_status_node",
        name="kvn_status_node",
        parameters=[LaunchConfiguration("status_config")],
        condition=IfCondition(LaunchConfiguration("use_status")),
    )
    return LaunchDescription([use_status_arg, status_config_arg, status_node])
