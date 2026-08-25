from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    odometry_node = Node(
        package='kvn_odometry',
        executable='odometry_node',
        name='odometry_node',
        parameters=[
            {'wheel_radius': 0.048},
            {'track_width': 0.205},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'use_imu_yaw': True},
            {'update_rate': 50.0}
        ]
    )
    return LaunchDescription([odometry_node])
