from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, PythonExpression
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
    
    drone_name_vicon = EnvironmentVariable(
        'DRONE_NAME_VICON',
        default_value='multilift_0'   # change this default if you like
    )
    
    vicon_topic_name = ['/vrpn_mocap/', drone_name_vicon, '/pose']

    drone_id = EnvironmentVariable(
        'DRONE_ID',
        default_value='0'   # change this default if you like
    )

    px4_topic_name_prefix = PythonExpression([
        "'' if '", drone_id, "' == '0' else '/px4_' + '", drone_id, "'"
    ])

    px4_topic_name = [px4_topic_name_prefix, '/fmu/in/vehicle_visual_odometry']


    vicon_px4_bridge_node = Node(
        package='vicon_px4_bridge',
        executable='vicon_px4_bridge_node',
        name='vicon_px4_bridge',   
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                'vicon_topic_name': vicon_topic_name,
                'px4_topic_name': px4_topic_name
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        vicon_px4_bridge_node,
    ])
