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
    
    input_frame_arg = DeclareLaunchArgument(
        'input_frame',
        default_value='',
        description='Input frame: ENU, FLU, or NED (overrides config file if set)'
    )
    
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='',
        description='Output frame: NED, ENU, or FLU (overrides config file if set)'
    )
    
    # Build parameters dictionary conditionally
    parameters = [LaunchConfiguration('params_file')]
    
    # Optional parameter overrides from command line
    param_overrides = {}
    
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
        input_frame_arg,
        output_frame_arg,
        vicon_px4_bridge_node
    ])
