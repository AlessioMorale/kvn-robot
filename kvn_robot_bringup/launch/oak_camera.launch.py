"""HARDWARE ONLY - OAK-D-Lite driver with point cloud on /oak/rgbd/points (the simulation bridges its own cloud)."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            'params_file': PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'oak.yaml']),
        }.items()
    )

    return LaunchDescription([camera])
