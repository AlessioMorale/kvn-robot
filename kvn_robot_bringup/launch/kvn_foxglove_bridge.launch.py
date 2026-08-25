from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    asset_uri_allowlist_arg = DeclareLaunchArgument(
        'asset_uri_allowlist',
        default_value="['^package://(?:\\\\w+/)*[\\\\w-]+(?:\\\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro))?$']"
    )
    
    asset_uri_allowlist = LaunchConfiguration('asset_uri_allowlist')
    
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'asset_uri_allowlist': asset_uri_allowlist}]
    )
    
    return LaunchDescription([
        asset_uri_allowlist_arg,
        foxglove_bridge
    ])
