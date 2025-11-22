from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    """Launch trajectory test node for a single drone."""

    drone_id_env = EnvironmentVariable('DRONE_ID', default_value='0')
    
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value=drone_id_env,
        description='Drone ID'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('traj_test'),
            'config',
            'traj_params.yaml'
        ]),
        description='Path to the configuration YAML file'
    )
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    config_file = LaunchConfiguration('config_file')
    
    # Trajectory test node
    traj_test_node = Node(
        package='traj_test',
        executable='traj_test_node',
        name=['traj_test_node_', drone_id],
        namespace='',
        parameters=[config_file, {'drone_id': drone_id}],
        output='screen',
        emulate_tty=True,
        arguments=[drone_id]
    )
    
    return LaunchDescription([
        drone_id_arg,
        config_file_arg,
        traj_test_node,
    ])