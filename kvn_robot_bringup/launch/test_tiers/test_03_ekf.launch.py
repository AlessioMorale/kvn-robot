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
    kvn_ekf_dir = FindPackageShare('kvn_ekf')
    
    test_02_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'test_tiers', 'test_02_odometry.launch.py'])
    ekf_launch = PathJoinSubstitution([kvn_ekf_dir, 'launch', 'ekf.launch.py'])
    
    tier_2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_02_launch),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )
    
    ekf_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ekf_launch)
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        tier_2_include,
        ekf_include
    ])
