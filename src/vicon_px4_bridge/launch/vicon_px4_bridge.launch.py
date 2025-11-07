from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('vicon_px4_bridge')
    
    # Default parameters file path
    default_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'vicon_px4_params.yaml'
    ])
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file'
    )
    
    vicon_topic_arg = DeclareLaunchArgument(
        'vicon_topic',
        default_value='',
        description='Vicon topic name (overrides config file if set)'
    )
    
    px4_topic_arg = DeclareLaunchArgument(
        'px4_topic',
        default_value='',
        description='PX4 topic name (overrides config file if set)'
    )
    
    # Position origin frame argument
    position_origin_frame_arg = DeclareLaunchArgument(
        'position_origin_frame',
        default_value='',
        description='Position origin frame: ENU, FLU, NED, or FRD (overrides config file if set)'
    )
    
    # Orientation origin frame argument
    orientation_origin_frame_arg = DeclareLaunchArgument(
        'orientation_origin_frame',
        default_value='',
        description='Orientation origin frame: ENU, FLU, NED, or FRD (overrides config file if set)'
    )
    
    # Position transform frame argument
    position_transform_frame_arg = DeclareLaunchArgument(
        'position_transform_frame',
        default_value='',
        description='Position transform frame: ENU, FLU, NED, or FRD (overrides config file if set)'
    )
    
    # Orientation transform frame argument
    orientation_transform_frame_arg = DeclareLaunchArgument(
        'orientation_transform_frame',
        default_value='',
        description='Orientation transform frame: ENU, FLU, NED, or FRD (overrides config file if set)'
    )
    
    # Build parameters dictionary conditionally
    parameters = [LaunchConfiguration('params_file')]
    
    # Node
    vicon_px4_bridge_node = Node(
        package='vicon_px4_bridge',
        executable='vicon_px4_bridge_node',
        name='vicon_px4_bridge',
        output='screen',
        parameters=parameters,
        remappings=[],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        params_file_arg,
        vicon_topic_arg,
        px4_topic_arg,
        position_origin_frame_arg,
        orientation_origin_frame_arg,
        position_transform_frame_arg,
        orientation_transform_frame_arg,
        vicon_px4_bridge_node
    ])