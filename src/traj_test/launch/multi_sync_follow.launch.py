#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    default_drone_id = EnvironmentVariable('DRONE_ID', default_value='0')

    total_drones_arg = DeclareLaunchArgument(
        'total_drones',
        default_value='3',
        description='Total number of drones in the swarm'
    )

    drone_ids_arg = DeclareLaunchArgument(
        'drone_ids',
        default_value='0,1,2',
        description='Comma-separated list of all drone IDs'
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
    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_alt',
        default_value='0.4',
        description='Takeoff altitude to gate GOTO (meters, positive up)'
    )
    alt_tol_arg = DeclareLaunchArgument(
        'alt_tol',
        default_value='0.05',
        description='Altitude tolerance for GOTO gating (meters)'
    )

    return LaunchDescription([
        total_drones_arg,
        drone_ids_arg,
        csv_base_path_arg,
        timer_period_arg,
        yaw_setpoint_arg,
        takeoff_alt_arg,
        alt_tol_arg,
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    total_drones = int(LaunchConfiguration('total_drones').perform(context))
    drone_ids_str = LaunchConfiguration('drone_ids').perform(context)
    csv_base_path = LaunchConfiguration('csv_base_path').perform(context)
    timer_period = LaunchConfiguration('timer_period').perform(context)
    yaw_setpoint = LaunchConfiguration('yaw_setpoint').perform(context)
    takeoff_alt = float(LaunchConfiguration('takeoff_alt').perform(context))
    alt_tol = float(LaunchConfiguration('alt_tol').perform(context))

    drone_ids = [int(x.strip()) for x in drone_ids_str.split(',')]
    if len(drone_ids) != total_drones:
        raise ValueError(f"Mismatch: drone_ids has {len(drone_ids)} IDs but total_drones={total_drones}")

    this_drone_id = int(os.environ.get('DRONE_ID', '0'))
    if this_drone_id not in drone_ids:
        raise ValueError(f"DRONE_ID={this_drone_id} not found in drone_ids={drone_ids}")

    nodes = []

    # Per-drone follower
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

    # Coordinator handling GOTO then TRAJ for the swarm
    coordinator_node = Node(
        package='traj_test',
        executable='swarm_goto_coordinator_node',
        name=f'swarm_goto_coordinator_{this_drone_id}',
        namespace='',
        parameters=[{
            'total_drones': total_drones,
            'drone_ids': drone_ids,
            'takeoff_alt': takeoff_alt,
            'alt_tol': alt_tol,
        }],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(coordinator_node)

    nodes.append(
        LogInfo(msg=f'Drone {this_drone_id}: launching follow_traj + goto/TRAJ coordinator '
                    f'(swarm: {drone_ids}, total: {total_drones})')
    )

    return nodes
