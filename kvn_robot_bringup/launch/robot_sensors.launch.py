"""HARDWARE ONLY - planar lidar (RPLIDAR C1) and OAK-D-Lite depth camera. Not used in the simulation,
which bridges its own scan and point cloud."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from kvn_robot_bringup import robot_config


def generate_launch_description():
    slam_backend_arg = DeclareLaunchArgument(
        'slam_backend',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'orbslam3'],
        description='SLAM backend; forwarded to oak_camera.launch.py for sensor rate/resolution tuning',
    )

    # rplidar_driver is a lifecycle node: its launch file configures and activates it.
    # Planar lidar in the URDF frame (lidar_attachment_link), so publish_tf:=false.
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rplidar_driver'), 'launch', 'rplidar_with_args.launch.py'])),
        launch_arguments={
            # serial_port, serial_baudrate from robot.yaml
            **robot_config.lidar_launch_arguments(robot_config.load()),
            'frame_id': 'lidar_attachment_link',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
            'publish_tf': 'false',
        }.items())

    depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'launch', 'oak_camera.launch.py'])),
        launch_arguments={'slam_backend': LaunchConfiguration('slam_backend')}.items())

    # Scoped: the driver launch files declare their own arguments (params_file, ...)
    return LaunchDescription([
        slam_backend_arg,
        GroupAction([rplidar], scoped=True),
        GroupAction([depth_camera], scoped=True),
    ])
