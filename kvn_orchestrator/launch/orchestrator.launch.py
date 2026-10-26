from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "orchestrator_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_orchestrator"), "config", "orchestrator.yaml"]
        ),
        description="Behaviour-tree orchestrator parameters",
    )
    tree_arg = DeclareLaunchArgument(
        "orchestrator_tree",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kvn_orchestrator"), "trees", "kvn_orchestrator.xml"]
        ),
        description="BehaviorTree.CPP XML file defining the orchestrator behaviour",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    use_orchestrator_arg = DeclareLaunchArgument(
        "use_orchestrator",
        default_value="true",
        description="Start the robot behaviour-tree orchestrator",
    )
    node = Node(
        package="kvn_orchestrator",
        executable="kvn_orchestrator_node",
        name="kvn_orchestrator_node",
        output="screen",
        parameters=[
            LaunchConfiguration("orchestrator_config"),
            {"tree_file": LaunchConfiguration("orchestrator_tree")},
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration("use_orchestrator")),
    )
    return LaunchDescription([config_arg, tree_arg, use_sim_time_arg, use_orchestrator_arg, node])
