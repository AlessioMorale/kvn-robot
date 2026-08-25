import os
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # Arguments
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware interfaces'
    )
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    # Paths
    kvn_robot_description_dir = FindPackageShare('kvn_robot_description')
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    
    urdf_file = PathJoinSubstitution([kvn_robot_description_dir, 'urdf', 'kvn_rover.urdf.xacro'])
    controllers_file = PathJoinSubstitution([kvn_robot_bringup_dir, 'config', 'controllers.yaml'])
    joy_launch_file = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'joy_to_twist.launch.py'])
    
    # Robot description
    robot_description_content = Command(['xacro ', urdf_file, ' use_mock_hardware:=', use_mock_hardware])
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}, controllers_file]
    )
    
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '20']
    )
    
    spawner_ssc = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_ssc',
        output='screen',
        arguments=['skid_steer_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '20']
    )
    
    # Includes
    joy_to_twist_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(joy_launch_file)
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        robot_state_publisher,
        controller_manager,
        spawner_jsb,
        spawner_ssc,
        joy_to_twist_include
    ])
