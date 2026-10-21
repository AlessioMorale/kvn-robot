"""HARDWARE ONLY - planar lidar (RPLIDAR C1) and OAK-D-Lite depth camera. Not used in the simulation,
which bridges its own scan and point cloud."""
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # rplidar_driver is a lifecycle node: its launch file configures and activates it.
    # Planar lidar in the URDF frame (lidar_attachment_link), so publish_tf:=false.
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rplidar_driver'), 'launch', 'rplidar_with_args.launch.py'])),
        launch_arguments={
            'serial_port': '/dev/ttyAMA2',
            'serial_baudrate': '460800',
            'frame_id': 'lidar_attachment_link',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
            'publish_tf': 'false',
        }.items())

    depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'launch', 'oak_camera.launch.py'])))

    # Scoped: the driver launch files declare their own arguments (params_file, ...)
    return LaunchDescription([
        GroupAction([rplidar], scoped=True),
        GroupAction([depth_camera], scoped=True),
    ])
