"""Autonomous frontier exploration (explore_lite). Start it after nav_stack / sim_nav_stack.

Needs slam_toolbox (/map, map -> odom) and Nav2 (/navigate_to_pose) already running; it is not part
of either stack so the rover can be driven manually or by goals without it. Works on the real robot
and in the simulation (use_sim_time:=true).

  Pause / resume:  ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use /clock (set true in the simulation)')

    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'explore.yaml']),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
        ]
    )

    return LaunchDescription([use_sim_time_arg, explore])
