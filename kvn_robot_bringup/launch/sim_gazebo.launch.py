from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    kvn_robot_bringup_dir = FindPackageShare('kvn_robot_bringup')
    kvn_robot_description_dir = FindPackageShare('kvn_robot_description')
    
    world_file = PathJoinSubstitution([kvn_robot_description_dir, 'worlds', 'kvn_robot.world'])
    urdf_file = PathJoinSubstitution([kvn_robot_description_dir, 'urdf', 'kvn_rover.urdf.xacro'])
    
    resource_path = PathJoinSubstitution([kvn_robot_description_dir, 'worlds']) # Needs to be appended with meshes somehow but usually ok if setup properly
    # Using SetEnvironmentVariable for GZ_SIM_RESOURCE_PATH might be better handled in shell, but we can do it:
    
    gazebo_server = Node(
        package='gz_sim',
        executable='gz',
        arguments=['sim', '-r', '-v', '4', world_file],
        name='gazebo',
        output='screen'
    )
    
    gazebo_client = Node(
        package='gz_sim',
        executable='gz',
        arguments=['sim', '-g'],
        name='gazebo_gui',
        output='screen'
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'kvn_rover', '-x', '0', '-y', '0', '-z', '0.1', '-file', urdf_file, 'sim_gazebo:=true'],
        output='screen'
    )
    
    robot_description_content = Command(['xacro ', urdf_file, ' sim_gazebo:=true'])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str), 'use_sim_time': use_sim_time}],
        remappings=[('joint_states', 'joint_states')]
    )
    
    controller_launch = PathJoinSubstitution([kvn_robot_bringup_dir, 'launch', 'controller.launch.py'])
    controller_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch),
        launch_arguments={'use_mock_hardware': 'false', 'use_sim_time': use_sim_time}.items()
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_robot,
        robot_state_publisher,
        controller_include
    ])
