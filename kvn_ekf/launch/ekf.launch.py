from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_default = PathJoinSubstitution([FindPackageShare('kvn_ekf'), 'config', 'fuse_ekf.yaml'])
    config_file_arg = DeclareLaunchArgument('config_file', default_value=config_file_default)

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    fusion_optimizer = Node(
        package='fuse_optimizers',
        executable='fixed_lag_smoother_node',
        name='fixed_lag_smoother_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([config_file_arg, use_sim_time_arg, fusion_optimizer])
