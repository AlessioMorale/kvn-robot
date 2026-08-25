from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    kvn_ekf_dir = FindPackageShare('kvn_ekf')
    
    sim_gazebo_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'sim_gazebo.launch.py'])
    odometry_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'odometry.launch.py'])
    ekf_launch = PathJoinSubstitution([kvn_ekf_dir, 'launch', 'ekf.launch.py'])
    slam_ekf_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'slam_ekf.launch.py'])
    
    sim_gazebo_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_gazebo_launch))
    odometry_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(odometry_launch))
    ekf_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(ekf_launch))
    slam_ekf_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_ekf_launch))
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server'
    )
    
    return LaunchDescription([
        sim_gazebo_include,
        odometry_include,
        ekf_include,
        slam_ekf_include,
        rviz_node,
        map_saver_server
    ])
