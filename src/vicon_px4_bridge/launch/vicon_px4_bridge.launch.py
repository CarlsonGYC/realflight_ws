from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile 

def generate_launch_description():


    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vicon_px4_bridge'),
            'config',
            'vicon_px4_params.yaml'
        ]),
        description='Path to the configuration YAML file'
    )

    config_file = LaunchConfiguration('config_file')

    vicon_px4_bridge_node = Node(
        package='vicon_px4_bridge',
        executable='vicon_px4_bridge_node',
        name='vicon_px4_bridge',   
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True)
        ],
    )

    return LaunchDescription([
        config_file_arg,
        vicon_px4_bridge_node,
    ])
