from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_rc_receiver_arg = DeclareLaunchArgument(
        'use_rc_receiver', default_value='true',
        description='Start the ELRS/CRSF receiver node publishing /joy'
    )
    teleop_config = PathJoinSubstitution([FindPackageShare('kvn_robot_bringup'), 'config', 'teleop.yaml'])

    crsf_joy_node = Node(
        package='elrs_joy_crsf_node',
        executable='crsf_node',
        name='crsf_joy_node',
        parameters=[teleop_config],
        condition=IfCondition(LaunchConfiguration('use_rc_receiver'))
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_config]
    )
    return LaunchDescription([use_rc_receiver_arg, crsf_joy_node, teleop_twist_joy_node])
