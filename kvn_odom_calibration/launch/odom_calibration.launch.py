"""Odometry calibration run. Needs the drive base (robot_base / sim_headless) and the lidar running.

  ros2 launch kvn_odom_calibration odom_calibration.launch.py [apply_results:=true] [use_sim_time:=true]

The rover moves as soon as the start delay is over. The node exits when done.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('kvn_odom_calibration'), 'config', 'odom_calibration.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'reference', default_value='scan',
            description="'scan' (lidar ICP) or 'tf' (reference_frame, e.g. slam_toolbox map)"),
        DeclareLaunchArgument('reference_frame', default_value='map'),
        DeclareLaunchArgument(
            'apply_results', default_value='false',
            description='Set the fitted parameters on the controller and odometry node at runtime'),
    ]

    calibration = Node(
        package='kvn_odom_calibration',
        executable='odom_calibration',
        name='odom_calibration',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             'reference': LaunchConfiguration('reference'),
             'reference_frame': LaunchConfiguration('reference_frame'),
             'apply_results': ParameterValue(LaunchConfiguration('apply_results'), value_type=bool)},
        ],
    )

    return LaunchDescription(args + [
        calibration,
        RegisterEventHandler(OnProcessExit(
            target_action=calibration, on_exit=[EmitEvent(event=Shutdown())])),
    ])
