#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    total_drones_arg = DeclareLaunchArgument(
        'total_drones',
        default_value='3',
        description='Total number of drones in the swarm'
    )

    drone_ids_arg = DeclareLaunchArgument(
        'drone_ids',
        default_value='0,1,2',
        description='Comma-separated list of drone IDs to coordinate'
    )

    total_drones = LaunchConfiguration('total_drones')
    drone_ids = LaunchConfiguration('drone_ids')

    # Use an OpaqueFunction to parse the comma-separated IDs with launch context
    from launch.actions import OpaqueFunction

    def build_node(context):
        ids = [int(x.strip()) for x in drone_ids.perform(context).split(',')]
        return [Node(
            package='traj_test',
            executable='swarm_goto_coordinator_node',
            name='swarm_goto_coordinator',
            output='screen',
            parameters=[{
                'total_drones': int(total_drones.perform(context)),
                'drone_ids': ids,
            }],
        )]

    return LaunchDescription([
        total_drones_arg,
        drone_ids_arg,
        OpaqueFunction(function=build_node),
    ])
