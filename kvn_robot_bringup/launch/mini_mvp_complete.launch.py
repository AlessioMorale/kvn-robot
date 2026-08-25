from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware', default_value='false'
    )
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    kvn_robot_description_dir = FindPackageShare('kvn_robot_description')
    urdf_file = PathJoinSubstitution([kvn_robot_description_dir, 'urdf', 'kvn_rover.urdf.xacro'])
    
    robot_description_content = Command(['xacro ', urdf_file, ' use_mock_hardware:=', use_mock_hardware])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )
    
    controller_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'controller.launch.py'])
    slam_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'slam.launch.py'])
    
    controller_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch)
    )
    
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch)
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        robot_state_publisher,
        controller_include,
        slam_include
    ])
