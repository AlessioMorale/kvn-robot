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
    
    test_03_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'test_tiers', 'test_03_ekf.launch.py'])
    slam_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'slam.launch.py'])
    
    tier_3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_03_launch),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )
    
    slam_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(slam_launch)
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        tier_3_include,
        slam_include
    ])
