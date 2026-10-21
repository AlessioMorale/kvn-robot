"""SIMULATION ONLY - headless simulation + the same fuse / slam_toolbox / Nav2 stack as the real robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    sim = include(
        'sim_headless.launch.py', gui=LaunchConfiguration('gui'), publish_odom_tf='false',
        world=LaunchConfiguration('world'), x=LaunchConfiguration('x'), y=LaunchConfiguration('y'))
    stack = include(
        'nav_stack.launch.py',
        use_sim_time='true', use_nav2=LaunchConfiguration('use_nav2'))

    return LaunchDescription([gui_arg, world_arg, x_arg, y_arg, use_nav2_arg, sim, stack])
