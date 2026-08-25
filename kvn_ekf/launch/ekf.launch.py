from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    enable_rviz_arg = DeclareLaunchArgument('enable_rviz', default_value='false')
    
    kvn_ekf_dir = FindPackageShare('kvn_ekf')
    config_file_default = PathJoinSubstitution([kvn_ekf_dir, 'config', 'fuse_ekf.yaml'])
    
    config_file_arg = DeclareLaunchArgument('config_file', default_value=config_file_default)
    config_file = LaunchConfiguration('config_file')
    
    fusion_optimizer = Node(
        package='fuse_optimizers',
        executable='fuse_optimizers_node',
        name='fusion_optimizer',
        output='screen',
        parameters=[{'config_file': config_file}]
    )
    
    return LaunchDescription([
        debug_arg,
        enable_rviz_arg,
        config_file_arg,
        fusion_optimizer
    ])
