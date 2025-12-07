#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    default_drone_id = os.environ.get('DRONE_ID', '0')
    # Declare launch arguments
    total_drones_arg = DeclareLaunchArgument(
        'total_drones',
        default_value='1',
        description='Total number of drones in the swarm'
    )

    drone_ids_arg = DeclareLaunchArgument(
        'drone_ids',
        default_value=default_drone_id,  # Default for single drone
        description='Comma-separated list of drone IDs' # (e.g., "1,2,3" or "10,20,30")
    )
    
    csv_base_path_arg = DeclareLaunchArgument(
        'csv_base_path',
        default_value='data/3drone_trajectories_new',
        description='Base path for CSV trajectory files (drone_X_traj_smoothed_100hz.csv)'
    )
    
    timer_period_arg = DeclareLaunchArgument(
        'timer_period',
        default_value='0.01',
        description='Timer period in seconds (default 100Hz)'
    )
    
    yaw_setpoint_arg = DeclareLaunchArgument(
        'yaw_setpoint',
        default_value='3.1415926',
        description='Yaw setpoint for all drones (radians)'
    )
    
    return LaunchDescription([
        total_drones_arg,
        drone_ids_arg,
        csv_base_path_arg,
        timer_period_arg,
        yaw_setpoint_arg,
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    total_drones = int(LaunchConfiguration('total_drones').perform(context))
    drone_ids_str = LaunchConfiguration('drone_ids').perform(context)
    csv_base_path = LaunchConfiguration('csv_base_path').perform(context)
    timer_period = LaunchConfiguration('timer_period').perform(context)
    yaw_setpoint = LaunchConfiguration('yaw_setpoint').perform(context)
    
    if drone_ids_str:
        drone_ids = [int(x.strip()) for x in drone_ids_str.split(',')]
    else:
        # Fallback to default sequential IDs
        drone_ids = list(range(total_drones))

    if len(drone_ids) != total_drones:
        raise ValueError(f"Number of drone_ids ({len(drone_ids)}) doesn't match total_drones ({total_drones})")

    nodes = []

    drone_id_env = EnvironmentVariable('DRONE_ID', default_value='0')
    
    # Create follow_traj node for this drone
    csv_path = f"{csv_base_path}/drone_{drone_id_env.perform(context)}_traj_smoothed_100hz.csv"
    
    node = Node(
        package='traj_test',
        executable='follow_traj_node',
        name=f'follow_traj_node_{drone_id_env.perform(context)}',
        namespace='',
        arguments=[str(drone_id_env.perform(context)), str(total_drones)],
        parameters=[{
            'timer_period': float(timer_period),
            'csv_path': csv_path,
            'yaw_setpoint': float(yaw_setpoint),
        }],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(node)
    
    # Add swarm coordinator node
    coordinator_node = Node(
        package='traj_test',
        executable='swarm_coordinator_node',
        name='swarm_coordinator',
        namespace='',
        arguments=[str(total_drones)],
        parameters=[{
            'drone_ids': drone_ids,  # Pass the list of drone IDs
        }],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(coordinator_node)
    
    # Add log messages
    nodes.append(
        LogInfo(msg=f'Launching {total_drones} follow_traj nodes (IDs: {drone_ids}) + coordinator')
    )
    
    return nodes
