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
    publish_joints_arg = DeclareLaunchArgument(
        'publish_joints', default_value='true'
    )
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    kvn_robot_description_dir = FindPackageShare('kvn_robot_description')
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    
    urdf_file = PathJoinSubstitution([kvn_robot_description_dir, 'urdf', 'kvn_rover.urdf.xacro'])
    controllers_file = PathJoinSubstitution([kvn_robot_bringup_dir, 'config', 'controllers.yaml'])
    robot_description_content = Command(['xacro ', urdf_file, ' use_mock_hardware:=', use_mock_hardware])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}, controllers_file]
    )
    
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--controller-manager', '/controller_manager', '--controller-manager-timeout', '20', 'joint_state_broadcaster']
    )
    
    spawner_ssc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--controller-manager', '/controller_manager', '--controller-manager-timeout', '20', 'skid_steer_controller']
    )
    
    hwmon_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'hwmon.launch.py'])
    odometry_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'odometry.launch.py'])
    teleop_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'teleop.launch.py'])
    status_launch = PathJoinSubstitution([FindPackageShare('kvn_status'), 'launch', 'kvn_status.launch.py'])
    
    hwmon_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(hwmon_launch))
    odometry_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(odometry_launch))
    teleop_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(teleop_launch))
    # /robot_status for the ELRS handset (reads /diagnostics, /battery_state, /joy)
    status_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(status_launch))
    
    return LaunchDescription([
        hwmon_include,
        use_mock_hardware_arg,
        publish_joints_arg,
        robot_state_publisher,
        controller_manager,
        spawner_jsb,
        spawner_ssc,
        odometry_include,
        teleop_include,
        status_include
    ])
