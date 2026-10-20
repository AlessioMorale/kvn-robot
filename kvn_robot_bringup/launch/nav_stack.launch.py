"""fuse odometry + lidar + slam_toolbox + depth camera + Nav2. Hardware by default; with
use_sim_time:=true use_hw_sensors:=false it runs on top of the simulation (see sim_nav_stack.launch.py)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Start Nav2 (planner/controller with STVL costmaps) on top of SLAM')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (simulation)')
    use_hw_sensors_arg = DeclareLaunchArgument(
        'use_hw_sensors', default_value='true',
        description='Start the real lidar and depth camera drivers (false in simulation)')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hw_sensors = IfCondition(LaunchConfiguration('use_hw_sensors'))

    def include(package, *path, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), *path])),
            launch_arguments=args.items())

    # odom -> base_link: wheel odometry (/odom) + IMU (/imu) fused by fuse, published on /odom_ekf
    ekf = include('kvn_ekf', 'launch', 'ekf.launch.py', use_sim_time=use_sim_time)

    # Planar lidar in the URDF frame (lidar_attachment_link), so no extra static transform
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_attachment_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        condition=hw_sensors
    )

    # map -> odom: slam_toolbox online async (pose graph, loop closure, Ceres solver).
    # slam_toolbox is a lifecycle node: its own launch file configures and activates it.
    slam_toolbox = include(
        'slam_toolbox', 'launch', 'online_async_launch.py',
        slam_params_file=PathJoinSubstitution([bringup, 'config', 'slam', 'slam_toolbox_ekf.yaml']),
        use_sim_time=use_sim_time)

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'depth_camera.launch.py'])),
        condition=hw_sensors)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'nav2.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    return LaunchDescription([
        use_nav2_arg,
        use_sim_time_arg,
        use_hw_sensors_arg,
        ekf,
        sllidar_node,
        slam_toolbox,
        map_saver_server,
        depth_camera,
        nav2,
    ])
