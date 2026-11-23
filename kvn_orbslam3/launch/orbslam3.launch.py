"""Launch ORB-SLAM3 (RGB-D + IMU) with map->odom TF bridge and 2D occupancy grid mapper."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('kvn_orbslam3')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    vocab_file_arg = DeclareLaunchArgument(
        'vocab_file',
        default_value='/opt/orbslam3/Vocabulary/ORBvoc.txt',
        description='Path to ORB-SLAM3 vocabulary file (ORBvoc.txt)',
    )
    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value='',
        description='Optional ORB-SLAM3 YAML override; empty auto-selects Pi 5 or sim config',
    )
    orbslam3_package_arg = DeclareLaunchArgument(
        'orbslam3_package', default_value='orbslam3',
        description='ROS 2 wrapper package providing the ORB-SLAM3 node',
    )
    orbslam3_executable_arg = DeclareLaunchArgument(
        'orbslam3_executable', default_value='rgbd-inertial',
        description='ORB-SLAM3 ROS 2 wrapper executable name',
    )
    launch_orbslam3_arg = DeclareLaunchArgument(
        'launch_orbslam3', default_value='true',
        description='Start the ORB-SLAM3 C++ node (false to run only bridge + mapper)',
    )
    enable_occupancy_mapper_arg = DeclareLaunchArgument(
        'enable_occupancy_mapper', default_value='true',
        description='Publish /map from /scan + ORB-SLAM3 TF for Nav2 static_layer / explore_lite',
    )
    rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu')
    pose_topic_arg = DeclareLaunchArgument('pose_topic', default_value='/orbslam3/camera_pose')

    use_sim_time = LaunchConfiguration('use_sim_time')

    pi5_settings = PathJoinSubstitution([pkg, 'config', 'oak_d_lite_rgbd_inertial_pi5.yaml'])
    sim_settings = PathJoinSubstitution([pkg, 'config', 'sim_rgbd_inertial.yaml'])
    resolved_settings = PythonExpression([
        "'", LaunchConfiguration('settings_file'), "' or (",
        "'", sim_settings, "' if '", use_sim_time, "'.lower() == 'true' else '", pi5_settings, "'",
        ")",
    ])
    resolved_rgb = PythonExpression([
        "'", LaunchConfiguration('rgb_topic'), "' or (",
        "'/camera/image' if '", use_sim_time, "'.lower() == 'true' else '/oak/rgb/image_raw'",
        ")",
    ])
    resolved_depth = PythonExpression([
        "'", LaunchConfiguration('depth_topic'), "' or (",
        "'/camera/depth/image' if '", use_sim_time, "'.lower() == 'true' else '/oak/stereo/image_raw'",
        ")",
    ])

    orbslam3_node = Node(
        package=LaunchConfiguration('orbslam3_package'),
        executable=LaunchConfiguration('orbslam3_executable'),
        name='orbslam3_rgbd_inertial',
        output='screen',
        arguments=[LaunchConfiguration('vocab_file'), resolved_settings],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('camera/rgb', resolved_rgb),
            ('camera/depth', resolved_depth),
            ('rgb/image_raw', resolved_rgb),
            ('depth/image_raw', resolved_depth),
            ('imu', LaunchConfiguration('imu_topic')),
            ('camera_pose', LaunchConfiguration('pose_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('launch_orbslam3')),
    )

    ros_params = PathJoinSubstitution([pkg, 'config', 'orbslam3_ros.yaml'])

    tf_bridge = Node(
        package='kvn_orbslam3',
        executable='orbslam3_tf_bridge',
        name='orbslam3_tf_bridge',
        output='screen',
        parameters=[
            ros_params,
            {'use_sim_time': use_sim_time, 'pose_topic': LaunchConfiguration('pose_topic')},
        ],
    )

    occupancy_mapper = Node(
        package='kvn_orbslam3',
        executable='orbslam3_occupancy_mapper',
        name='orbslam3_occupancy_mapper',
        output='screen',
        parameters=[ros_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_occupancy_mapper')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        vocab_file_arg,
        settings_file_arg,
        orbslam3_package_arg,
        orbslam3_executable_arg,
        launch_orbslam3_arg,
        enable_occupancy_mapper_arg,
        rgb_topic_arg,
        depth_topic_arg,
        imu_topic_arg,
        pose_topic_arg,
        orbslam3_node,
        tf_bridge,
        occupancy_mapper,
    ])
