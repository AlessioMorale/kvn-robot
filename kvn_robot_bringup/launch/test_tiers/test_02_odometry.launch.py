import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware interfaces'
    )
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    
    test_01_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'test_tiers', 'test_01_locomotion.launch.py'])
    odometry_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'odometry.launch.py'])
    
    tier_1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_01_launch),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )
    
    odometry_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(odometry_launch)
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        tier_1_include,
        odometry_include
    ])
