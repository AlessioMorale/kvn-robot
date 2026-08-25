from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_to_twist_node = Node(
        package='kvn_joy_teleop',
        executable='joy_to_twist_node',
        name='joy_to_twist',
        parameters=[
            {'max_linear_vel': 1.0},
            {'max_angular_vel': 2.0},
            {'linear_axis': 1},
            {'angular_axis': 3},
            {'linear_inverted': False},
            {'angular_inverted': False}
        ]
    )
    return LaunchDescription([joy_to_twist_node])
