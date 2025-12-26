#!/usr/bin/env python3
"""
Launch the HITL geometric multilift controller with swarm GOTO/TRAJ coordinator.
Payload pose comes from /payload_odom, drone position from /simulation/position_drone_<i>,
and attitude/body rates from PX4 odometry.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_total = os.environ.get('TOTAL_DRONES', '3')
    default_params = PathJoinSubstitution([
        FindPackageShare("geom_multilift"),
        "config",
        "geom_multilift_hitl_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'total_drones',
            default_value=str(default_total),
            description='Total number of drones in the swarm'
        ),
        DeclareLaunchArgument(
            'drone_ids',
            default_value='0,1,2',
            description='Comma-separated list of all drone IDs'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with geom_multilift HITL parameters'
        ),
        DeclareLaunchArgument(
            'takeoff_alt',
            default_value='0.4',
            description='Takeoff altitude to gate GOTO (meters, positive up)'
        ),
        DeclareLaunchArgument(
            'alt_tol',
            default_value='0.05',
            description='Altitude tolerance for GOTO gating (meters)'
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    total_drones = int(LaunchConfiguration('total_drones').perform(context))
    drone_ids_str = LaunchConfiguration('drone_ids').perform(context)
    params_file = LaunchConfiguration('params_file')
    takeoff_alt = float(LaunchConfiguration('takeoff_alt').perform(context))
    alt_tol = float(LaunchConfiguration('alt_tol').perform(context))

    drone_ids = [int(x.strip()) for x in drone_ids_str.split(',')]
    if len(drone_ids) != total_drones:
        raise ValueError(
            f"Mismatch: drone_ids has {len(drone_ids)} IDs but total_drones={total_drones}"
        )

    this_drone_id = int(os.environ.get('DRONE_ID', '0'))
    if this_drone_id not in drone_ids:
        raise ValueError(
            f"DRONE_ID={this_drone_id} not found in drone_ids={drone_ids}"
        )

    nodes = []

    geom_node = Node(
        package='geom_multilift',
        executable='geom_multilift_hitl_node',
        name=f'geom_multilift_hitl_node_{this_drone_id}',
        namespace='',
        arguments=[str(this_drone_id), str(total_drones)],
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )
    nodes.append(geom_node)

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
        LogInfo(
            msg=(
                f'Drone {this_drone_id}: launching geom_multilift_hitl + goto/TRAJ coordinator '
                f'(swarm: {drone_ids}, total: {total_drones})'
            )
        )
    )

    return nodes
