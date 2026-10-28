"""Real robot, everything: drive base, sensors (lidar, camera), SLAM and Nav2.

For running by hand. On the robot systemd starts the layers as separate units instead
(kvn-control, kvn-sensors, kvn-nav; see systemd/README.md). Each layer is its own launch file and runs
standalone:
  robot_base     robot_state_publisher, ros2_control, wheel odometry, IMU, teleop, hwmon, status
  robot_sensors  lidar + OAK depth camera
  nav_stack      fuse (odom -> base_link), slam_toolbox (map -> odom), Nav2, orchestrator
No node is started by more than one of them. For the simulation use sim_nav_stack.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    def include(name, **args):
        # Scoped: launch arguments declared by an included file (e.g. params_file) must not leak
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', name])),
            launch_arguments=args.items())], scoped=True)

    use_mock_hardware_arg = DeclareLaunchArgument('use_mock_hardware', default_value='false')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true', description='Start Nav2 on top of fuse + slam_toolbox')
    use_wled_arg = DeclareLaunchArgument(
        'use_wled', default_value='false', description='Start the WLED signalling light')
    wled_port_arg = DeclareLaunchArgument(
        'wled_port', default_value='/dev/ttyACM0',
        description='Serial device connected to the WLED ESP32')
    use_display_arg = DeclareLaunchArgument(
        'use_display', default_value='false', description='Start the status display')
    use_orchestrator_arg = DeclareLaunchArgument(
        'use_orchestrator', default_value='true',
        description='Start the behaviour-tree orchestrator (requires Nav2)')

    return LaunchDescription([
        use_mock_hardware_arg,
        use_nav2_arg,
        use_wled_arg,
        wled_port_arg,
        use_display_arg,
        use_orchestrator_arg,
        include('robot_base.launch.py',
                use_mock_hardware=LaunchConfiguration('use_mock_hardware'),
                use_wled=LaunchConfiguration('use_wled'),
                wled_port=LaunchConfiguration('wled_port'),
                use_display=LaunchConfiguration('use_display')),
        include('robot_sensors.launch.py'),
        include('nav_stack.launch.py',
                use_nav2=LaunchConfiguration('use_nav2'),
                use_orchestrator=LaunchConfiguration('use_orchestrator')),
    ])
