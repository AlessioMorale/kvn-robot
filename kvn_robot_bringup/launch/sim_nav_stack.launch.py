"""SIMULATION ONLY - headless simulation + the same fuse / slam_toolbox / Nav2 stack as the real robot.

Nav2 drives through the same teleop_twist_joy_node (kvn_joy_teleop) as on the robot. There is no
RC receiver here, so it forwards Nav2's command without requiring Joy or the orchestrator's
permit (set require_autonomy_permit:=true and publish /joy, axes[4] = 1.0 for autonomous, to
exercise the orchestrator's modes; the orchestrator only starts then, see use_orchestrator).
/robot_status is not published, so the orchestrator skips its robot-fault checks.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('kvn_robot_bringup')

    def include(name, **args):
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', name])),
            launch_arguments=args.items())], scoped=True)

    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    world_arg = DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
        FindPackageShare('kvn_robot_description'), 'worlds', 'kvn_turtlebot3_world.world']))
    x_arg = DeclareLaunchArgument('x', default_value='-2.0')
    y_arg = DeclareLaunchArgument('y', default_value='-0.5')
    use_nav2_arg = DeclareLaunchArgument('use_nav2', default_value='true')
    require_permit_arg = DeclareLaunchArgument(
        'require_autonomy_permit', default_value='false',
        description='Forward Nav2 commands only while the orchestrator permits it (needs /joy)')
    # Without /joy the orchestrator never leaves `initialising` (waiting_for_joy) and, while its
    # gate is closed, pauses explore_lite. So it only runs when the permit is exercised.
    use_orchestrator_arg = DeclareLaunchArgument(
        'use_orchestrator', default_value=LaunchConfiguration('require_autonomy_permit'),
        description='Start the orchestrator (default: same as require_autonomy_permit; needs /joy)')

    sim = include(
        'sim_headless.launch.py', gui=LaunchConfiguration('gui'), publish_odom_tf='false',
        world=LaunchConfiguration('world'), x=LaunchConfiguration('x'), y=LaunchConfiguration('y'))
    stack = include(
        'nav_stack.launch.py',
        use_sim_time='true', use_nav2=LaunchConfiguration('use_nav2'),
        use_orchestrator=LaunchConfiguration('use_orchestrator'), require_robot_status='false')
    require_permit = ParameterValue(LaunchConfiguration('require_autonomy_permit'), value_type=bool)
    teleop = Node(
        package='kvn_joy_teleop', executable='teleop_override_node', name='teleop_twist_joy_node',
        parameters=[
            PathJoinSubstitution([bringup, 'config', 'teleop.yaml']),
            {'use_sim_time': True, 'require_joy_for_auto': require_permit,
             'require_autonomy_permit': require_permit}])

    return LaunchDescription([
        gui_arg, world_arg, x_arg, y_arg, use_nav2_arg, require_permit_arg, use_orchestrator_arg,
        sim, stack, teleop])
