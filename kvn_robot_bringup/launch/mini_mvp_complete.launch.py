"""Hardware bring-up (used by systemd kvn-control.service): drive, odometry, teleop, sensors, SLAM, Nav2.

Real robot only: starts the DDSM210 drive hardware, the ELRS receiver, the lidar and the OAK camera.
For the simulation use sim_nav_stack.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    def include(name, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', name])),
            launch_arguments=args.items())

    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='false')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true', description='Start Nav2 on top of fuse + slam_toolbox')

    # robot_state_publisher, ros2_control, odometry, teleop, hwmon, status
    controller = include('controller.launch.py', use_mock_hardware=LaunchConfiguration('use_mock_hardware'))
    # fuse (odom -> base_link), lidar, slam_toolbox (map -> odom), depth camera, Nav2
    nav_stack = include('nav_stack.launch.py', use_nav2=LaunchConfiguration('use_nav2'))

    return LaunchDescription([use_mock_hardware_arg, use_nav2_arg, controller, nav_stack])
