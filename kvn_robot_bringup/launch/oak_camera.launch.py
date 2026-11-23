"""HARDWARE ONLY - OAK-D-Lite driver with point cloud on /oak/rgbd/points (the simulation bridges its own cloud)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    slam_backend_arg = DeclareLaunchArgument(
        'slam_backend',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'orbslam3'],
        description='SLAM backend; orbslam3 selects 640x400 @ 15 Hz aligned RGB-D settings',
    )

    oak_params_file = PythonExpression([
        "'", PathJoinSubstitution([bringup, 'config', 'oak_orbslam3.yaml']),
        "' if '", LaunchConfiguration('slam_backend'), "' == 'orbslam3' else '",
        PathJoinSubstitution([bringup, 'config', 'oak.yaml']), "'",
    ])

    # The URDF already carries the camera frames (camera_name "oak"), so the driver must not
    # publish its own calibration TF.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('depthai_ros_driver'), 'launch', 'driver.launch.py'])),
        launch_arguments={
            'name': 'oak',
            'camera_model': 'OAK-D-LITE',
            'parent_frame': 'camera_attachment_link',
            'publish_tf_from_calibration': 'false',
            'pointcloud.enable': 'true',
            'params_file': oak_params_file,
        }.items()
    )

    return LaunchDescription([slam_backend_arg, camera])
