from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Parameters live in controllers.yaml (odometry_node section) so that wheel_radius,
    # track_width and chi stay in one place next to the skid_steer_controller values
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('kvn_robot_bringup'), 'config', 'controllers.yaml'])

    odometry_node = Node(
        package='kvn_odometry',
        executable='odometry_node',
        name='odometry_node',
        parameters=[controllers_file]
    )
    return LaunchDescription([odometry_node])
