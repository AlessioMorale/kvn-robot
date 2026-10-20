"""SIMULATION ONLY - headless gz-sim: world, rover, ros2_control (inside gz), sensor bridge, odometry.

Run on its own to drive the rover (teleop, `ros2 topic pub /cmd_vel`), or use
sim_nav_stack.launch.py for the full fuse + slam_toolbox + Nav2 stack on top.
"""
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')
    description = FindPackageShare('kvn_robot_description')

    # kvn_turtlebot3_world.world: TurtleBot3 World (walls + 3x3 pillars), spawn at (-2.0, -0.5).
    # kvn_robot.world: plain 10x10 m arena with a few boxes, spawn at (0, 0).
    world_arg = DeclareLaunchArgument(
        'world', default_value=PathJoinSubstitution([description, 'worlds', 'kvn_turtlebot3_world.world']))
    x_arg = DeclareLaunchArgument('x', default_value='-2.0', description='Spawn x [m]')
    y_arg = DeclareLaunchArgument('y', default_value='-0.5', description='Spawn y [m]')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false', description='Also start the gz GUI client (needs a display)')

    controllers_file = PathJoinSubstitution([bringup, 'config', 'controllers.yaml'])
    urdf_file = PathJoinSubstitution([description, 'urdf', 'kvn_rover.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_gazebo:=true simulation_controllers:=', controllers_file]),
        value_type=str)

    # Meshes are referenced as package://kvn_robot_description/...
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([description, '..']))
    # model://turtlebot3_world used by kvn_turtlebot3_world.world
    gz_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([FindPackageShare('nav2_minimal_tb3_sim'), 'models']))

    def gz_sim(gz_args, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': gz_args}.items(), **kwargs)

    # -s: server only, --headless-rendering: EGL offscreen rendering for the camera (no X needed)
    gz_server = gz_sim(['-r -s --headless-rendering -v 2 ', LaunchConfiguration('world')])
    # GUI client only
    gz_gui = gz_sim('-g', condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'kvn_rover', '-topic', 'robot_description', '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.1'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': PathJoinSubstitution([bringup, 'config', 'gz_bridge.yaml']),
                     'use_sim_time': True}])

    # The controller_manager lives inside gz_ros2_control; only the spawners run here
    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '30'],
            parameters=[{'use_sim_time': True}])

    # fuse publishes odom -> base_link in the nav stack; set publish_tf:=true to use wheel odometry
    # alone (without sim_nav_stack.launch.py).
    odometry_node = Node(
        package='kvn_odometry', executable='odometry_node', name='odometry_node',
        parameters=[controllers_file, {'use_sim_time': True},
                    {'publish_tf': LaunchConfiguration('publish_odom_tf')}])
    publish_odom_tf_arg = DeclareLaunchArgument('publish_odom_tf', default_value='false')

    return LaunchDescription([
        world_arg, x_arg, y_arg, gui_arg, publish_odom_tf_arg,
        gz_resource_path, gz_tb3_models, gz_server, gz_gui,
        robot_state_publisher, spawn_robot, bridge,
        spawner('joint_state_broadcaster'), spawner('skid_steer_controller'),
        odometry_node,
    ])
