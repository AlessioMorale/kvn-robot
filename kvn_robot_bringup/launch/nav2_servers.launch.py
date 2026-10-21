"""Nav2 navigation servers for the KVN rover (no map server/AMCL: slam_toolbox provides /map and map -> odom).

Started node by node instead of through nav2_bringup's navigation_launch.py, which also starts the
route, collision-monitor, docking and following servers that this rover has no configuration for
(one of them failing to configure stops the whole lifecycle bring-up).

Command chain: controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('nav2_params_file'),
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    tf_remaps = []
    nodes = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]
    actions = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}], remappings=remaps + tf_remaps)
        for pkg, exe, remaps in nodes
    ]
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': [exe for _, exe, _ in nodes]}]))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'nav2_params.yaml'])),
        *actions,
    ])
