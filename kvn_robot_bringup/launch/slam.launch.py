from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    slam_config_file = PathJoinSubstitution([kvn_robot_bringup_dir, 'config', 'slam', 'slam_toolbox_async.yaml'])

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {'channel_type': 'serial'},
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 115200},
            {'frame_id': 'laser'},
            {'inverted': False},
            {'angle_compensate': True},
            {'scan_mode': 'Standard'}
        ]
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config_file],
        remappings=[('scan', '/scan')]
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server'
    )

    return LaunchDescription([
        sllidar_node,
        base_to_laser_tf,
        slam_toolbox,
        map_saver_server
    ])
