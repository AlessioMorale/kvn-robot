"""RC teleop: ELRS receiver -> /joy -> teleop_twist_joy_node -> /cmd_vel.

teleop_twist_joy_node is kvn_joy_teleop's teleop_override_node: with the enable switch (AUX1) held
it drives from the sticks, otherwise it forwards Nav2's /cmd_vel_auto while the orchestrator
permits it. It is the operator override for every mode, one hop before the controller.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from kvn_robot_bringup import robot_config


def generate_launch_description():
    use_rc_receiver_arg = DeclareLaunchArgument(
        'use_rc_receiver', default_value='true',
        description='Start the ELRS/CRSF receiver node publishing /joy'
    )
    teleop_config = PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'teleop.yaml'])
    overrides = robot_config.node_overrides(robot_config.load())

    crsf_joy_node = Node(
        package='elrs_joy_crsf_node',
        executable='crsf_node',
        name='crsf_joy_node',
        parameters=[teleop_config, overrides['crsf_joy_node']],
        condition=IfCondition(LaunchConfiguration('use_rc_receiver'))
    )

    teleop_node = Node(
        package='kvn_joy_teleop',
        executable='teleop_override_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_config]
    )
    return LaunchDescription([use_rc_receiver_arg, crsf_joy_node, teleop_node])
