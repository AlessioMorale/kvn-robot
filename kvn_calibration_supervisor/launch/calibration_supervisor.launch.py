"""Calibration supervisor. Needs the drive base and the sensors (robot_base + robot_sensors, or
sim_headless); stop Nav2 and exploration, which also publish /cmd_vel.

  ros2 launch kvn_calibration_supervisor calibration_supervisor.launch.py [use_sim_time:=true]
  ros2 topic pub --once /calibration/command std_msgs/msg/String "{data: START}"
  ros2 topic echo /calibration/status

Nothing moves until START and then NEXT.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('kvn_calibration_supervisor'), 'config',
                 'calibration_supervisor.yaml'])),
        DeclareLaunchArgument(
            'odom_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('kvn_odom_calibration'), 'config', 'odom_calibration.yaml']),
            description='Parameters of the odometry step (the odom_calibration node)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'output_path', default_value='',
            description='Calibration file; empty: /etc/kvn/calibration_<hostname>.yaml'),
    ]
    supervisor = Node(
        package='kvn_calibration_supervisor',
        executable='calibration_supervisor',
        # No name=: it becomes a process-wide __node remap, which would also rename the
        # odometry step's node (odom_calibration) and make it read the wrong parameters.
        output='screen',
        emulate_tty=True,
        # Both files: the odometry step's node, created in this process, reads its own section
        parameters=[
            LaunchConfiguration('odom_params_file'),
            LaunchConfiguration('params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             'output_path': ParameterValue(LaunchConfiguration('output_path'), value_type=str)},
        ],
    )
    return LaunchDescription(args + [supervisor])
