#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    # Get drone ID from environment (each drone has its own DRONE_ID)
    default_drone_id = os.environ.get('DRONE_ID', '0')
    
    # Launch arguments
    total_drones_arg = DeclareLaunchArgument(
        'total_drones',
        default_value='3',
        description='Total number of drones in the swarm'
    )

    drone_ids_arg = DeclareLaunchArgument(
        'drone_ids',
        default_value='0,1,2',  # All drone IDs in the swarm
        description='Comma-separated list of all drone IDs (e.g., "0,1,2")'
    )
    
    csv_base_path_arg = DeclareLaunchArgument(
        'csv_base_path',
        default_value='data/3drone_trajectories_new',
        description='Base directory containing drone_X_traj_smoothed_100hz.csv files'
    )
    
    timer_period_arg = DeclareLaunchArgument(
        'timer_period',
        default_value='0.01',
        description='Control loop period in seconds (default 100Hz)'
    )
    
    yaw_setpoint_arg = DeclareLaunchArgument(
        'yaw_setpoint',
        default_value='3.1415926',
        description='Yaw setpoint in radians'
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
    # Parse launch arguments
    total_drones = int(LaunchConfiguration('total_drones').perform(context))
    drone_ids_str = LaunchConfiguration('drone_ids').perform(context)
    csv_base_path = LaunchConfiguration('csv_base_path').perform(context)
    timer_period = LaunchConfiguration('timer_period').perform(context)
    yaw_setpoint = LaunchConfiguration('yaw_setpoint').perform(context)
    
    # Parse drone IDs list
    drone_ids = [int(x.strip()) for x in drone_ids_str.split(',')]
    
    # Validate drone ID count
    if len(drone_ids) != total_drones:
        raise ValueError(
            f"Mismatch: drone_ids has {len(drone_ids)} IDs "
            f"but total_drones={total_drones}"
        )
    
    # Get THIS drone's ID from environment
    this_drone_id = int(os.environ.get('DRONE_ID', '0'))
    
    # Validate this drone is in the swarm
    if this_drone_id not in drone_ids:
        raise ValueError(
            f"DRONE_ID={this_drone_id} not found in drone_ids={drone_ids}"
        )
    
    nodes = []
    
    # === FOLLOW_TRAJ NODE (per-drone) ===
    # Each drone follows its own trajectory file: drone_{ID}_traj_smoothed_100hz.csv
    csv_path = f"{csv_base_path}/drone_{this_drone_id}_traj_smoothed_100hz.csv"
    
    follow_traj_node = Node(
        package='traj_test',
        executable='follow_traj_node',
        name=f'follow_traj_node_{this_drone_id}',
        namespace='',
        arguments=[str(this_drone_id), str(total_drones)],
        parameters=[{
            'timer_period': float(timer_period),
            'csv_path': csv_path,
            'yaw_setpoint': float(yaw_setpoint),
        }],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(follow_traj_node)
    
    # === SWARM COORDINATOR NODE (per-drone, but all identical) ===
    # Each drone runs a coordinator that monitors ALL drones and sends TRAJ command
    # NOTE: All coordinators will send the same command, this is redundant but safe
    coordinator_node = Node(
        package='traj_test',
        executable='swarm_coordinator_node',
        # Unique name per drone to avoid name collisions when running on multiple machines
        name=f'swarm_coordinator_{this_drone_id}',
        namespace='',
        arguments=[str(total_drones)],
        parameters=[{
            'drone_ids': drone_ids,  # Full list of all drone IDs
        }],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(coordinator_node)
    
    # Log startup info
    nodes.append(
        LogInfo(msg=f'Drone {this_drone_id}: Launching follow_traj + coordinator '
                    f'(swarm: {drone_ids}, total: {total_drones})')
    )
    
    return nodes
