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
    use_wled_arg = DeclareLaunchArgument(
        "use_wled",
        default_value="false",
        description="Start the WLED signalling-light node",
    )
    wled_config_arg = DeclareLaunchArgument(
        "wled_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_status"), "config", "kvn_wled_signal.yaml"]
        ),
        description="Parameters file for kvn_wled_signal",
    )
    wled_port_arg = DeclareLaunchArgument(
        "wled_port",
        default_value="/dev/ttyACM0",
        description="Serial device connected to the WLED ESP32",
    )

    status_node = Node(
        package="kvn_status",
        executable="kvn_status_node",
        name="kvn_status_node",
        parameters=[LaunchConfiguration("status_config")],
        condition=IfCondition(LaunchConfiguration("use_status")),
    )
    wled_node = Node(
        package="kvn_status",
        executable="kvn_wled_signal.py",
        name="kvn_wled_signal_node",
        parameters=[
            LaunchConfiguration("wled_config"),
            {"serial_port": LaunchConfiguration("wled_port")},
        ],
        condition=IfCondition(LaunchConfiguration("use_wled")),
    )
    return LaunchDescription([
        use_status_arg,
        status_config_arg,
        use_wled_arg,
        wled_config_arg,
        wled_port_arg,
        status_node,
        wled_node,
    ])
