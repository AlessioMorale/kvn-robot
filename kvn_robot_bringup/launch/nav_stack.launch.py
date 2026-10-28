"""fuse odometry + slam_toolbox + Nav2 + behaviour-tree orchestrator. Expects /odom, /imu and /scan (+ /oak/rgbd/points) to be
published: by base + sensors on the robot (see robot.launch.py), by sim_headless in the simulation
(see sim_nav_stack.launch.py, which passes use_sim_time:=true)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from kvn_robot_bringup import robot_config


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Start Nav2 (planner/controller with STVL costmaps) on top of SLAM')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (simulation)')
    use_orchestrator_arg = DeclareLaunchArgument(
        'use_orchestrator', default_value='true',
        description='Start the behaviour-tree orchestrator (requires Nav2)')
    require_robot_status_arg = DeclareLaunchArgument(
        'require_robot_status', default_value='true',
        description='Orchestrator blocks autonomy on /robot_status faults (false in simulation)')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Battery limits for the orchestrator's return-to-home from robot.yaml
    orchestrator_config = robot_config.merged_params_file(
        os.path.join(get_package_share_directory('kvn_orchestrator'), 'config', 'orchestrator.yaml'),
        'kvn_orchestrator_node',
        robot_config.node_overrides(robot_config.load())['kvn_orchestrator_node'])

    def include(package, *path, **args):
        # Scoped: launch arguments declared by an included file (e.g. params_file) must not leak
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), *path])),
            launch_arguments=args.items())], scoped=True)

    # odom -> base_link: wheel odometry (/odom) + IMU (/imu) fused by fuse, published on /odom_ekf
    ekf = include('kvn_ekf', 'launch', 'ekf.launch.py', use_sim_time=use_sim_time)

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

    nav2 = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'nav2_servers.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items())],
        scoped=True, condition=IfCondition(LaunchConfiguration('use_nav2')))

    orchestrator = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kvn_orchestrator'), 'launch', 'orchestrator.launch.py'])),
        launch_arguments={
            'use_orchestrator': LaunchConfiguration('use_orchestrator'),
            'use_sim_time': use_sim_time,
            'orchestrator_config': orchestrator_config,
            'require_robot_status': LaunchConfiguration('require_robot_status'),
        }.items())],
        scoped=True, condition=IfCondition(LaunchConfiguration('use_nav2')))

    return LaunchDescription([
        use_nav2_arg,
        use_sim_time_arg,
        use_orchestrator_arg,
        require_robot_status_arg,
        ekf,
        slam_toolbox,
        map_saver_server,
        nav2,
        orchestrator,
    ])
