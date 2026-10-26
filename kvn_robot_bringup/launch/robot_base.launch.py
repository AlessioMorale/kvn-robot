"""HARDWARE ONLY - drive base: robot_state_publisher, ros2_control, wheel odometry, IMU, teleop, hwmon, status."""
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
    use_wled_arg = DeclareLaunchArgument(
        'use_wled', default_value='false',
        description='Start the WLED signalling light (requires the serial adapter dependency)'
    )
    wled_port_arg = DeclareLaunchArgument(
        'wled_port', default_value='/dev/ttyACM0',
        description='Serial device connected to the WLED ESP32'
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
    
    # --param-file: the controller node does not inherit the controller_manager's parameters, without
    # it skid_steer_controller runs on its compiled-in defaults (wheel_radius 0.048, track_width 0.205)
    spawner_ssc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--controller-manager', '/controller_manager', '--controller-manager-timeout', '20',
                   '--param-file', controllers_file, 'skid_steer_controller']
    )
    
    def include(launch_file, **args):
        # Scoped: launch arguments declared by an included file (e.g. params_file) must not leak
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file), launch_arguments=args.items())], scoped=True)

    hwmon_include = include(PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'hardware_monitor.launch.py']))
    odometry_include = include(PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'wheel_odometry.launch.py']))
    teleop_include = include(PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'teleop.launch.py']))
    # /imu (ISM330DHCX on SPI0), read by wheel odometry and fuse
    imu_include = include(PathJoinSubstitution([FindPackageShare('ism330dhcx_node'), 'launch', 'ism330dhcx.launch.py']))
    # /robot_status for the ELRS handset (reads /diagnostics, /battery_state, /joy)
    status_include = include(
        PathJoinSubstitution([FindPackageShare('kvn_status'), 'launch', 'kvn_status.launch.py']),
        use_wled=LaunchConfiguration('use_wled'),
        wled_port=LaunchConfiguration('wled_port'))

    return LaunchDescription([
        hwmon_include,
        use_mock_hardware_arg,
        publish_joints_arg,
        use_wled_arg,
        wled_port_arg,
        robot_state_publisher,
        controller_manager,
        spawner_jsb,
        spawner_ssc,
        odometry_include,
        imu_include,
        teleop_include,
        status_include
    ])
