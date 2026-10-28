from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from kvn_robot_bringup import robot_config

def generate_launch_description():
    use_hwmon_arg = DeclareLaunchArgument(
        'use_hwmon', default_value='true',
        description='Start battery/thermal monitoring (needs the robot hwmon sysfs devices)'
    )
    hwmon_config = PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'hwmon.yaml'])

    hwmon_node = Node(
        package='hwmon_diagnostic_updater',
        executable='hwmon_diagnostic_updater_node',
        name='hwmon_diagnostic_updater_node',
        # Power monitor device and battery limits from robot.yaml
        parameters=[hwmon_config,
                    robot_config.node_overrides(robot_config.load())['hwmon_diagnostic_updater_node']],
        condition=IfCondition(LaunchConfiguration('use_hwmon'))
    )
    return LaunchDescription([use_hwmon_arg, hwmon_node])
