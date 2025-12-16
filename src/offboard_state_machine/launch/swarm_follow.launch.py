#!/usr/bin/env python3
"""
Single drone launch file for offboard FSM
Launches one drone that takes off and goes to the first point from a CSV trajectory file
"""

from pathlib import Path
import csv
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def read_first_trajectory_point(base_dir: str, drone_id: str):
    """
    Read the first point from the drone's trajectory CSV file.

    Args:
        base_dir: Directory containing trajectory CSVs
        drone_id: The drone ID (e.g., '0', '1', '2')

    Returns:
        tuple: (x, y, z) position of the first trajectory point
    """
    csv_path = Path(base_dir) / f"drone_{drone_id}_traj_smoothed_100hz.csv"

    if not csv_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {csv_path}")

    with csv_path.open('r') as csvfile:
        csv_reader = csv.reader(csvfile)

        # Skip header if exists
        header = next(csv_reader)

        # Read first data row
        first_row = next(csv_reader)

        # Assuming CSV format: time, x, y, z, ...
        x = float(first_row[1])
        y = float(first_row[2])
        z = float(first_row[3])

        return x, y, z


def launch_setup(context):
    drone_id_value = LaunchConfiguration('drone_id').perform(context)
    drone_id_int = int(drone_id_value)
    traj_base_dir = LaunchConfiguration('traj_base_dir').perform(context)
    goto_x, goto_y, goto_z = read_first_trajectory_point(traj_base_dir, drone_id_value)

    # Calculate takeoff altitude from z position (NED frame: negative = up)
    takeoff_altitude = abs(goto_z)

    fsm_node = Node(
        package="offboard_state_machine",
        executable="offboard_fsm_node",
        name=f"offboard_fsm_node_{drone_id_value}",
        output="screen",
        parameters=[{
            "drone_id": drone_id_int,
            "takeoff_alt": takeoff_altitude,
            "takeoff_time": 10.0,
            "climb_rate": 1.0,
            "landing_time": 2.0,

            # GOTO target (first CSV point)
            'goto_x': goto_x,
            'goto_y': goto_y,
            'goto_z': goto_z,
            'goto_tol': 0.05,
            'goto_max_vel': 1.0,
            'goto_accel_time': 4.0,

            # Takeoff position (use payload_offset with inward_offset=0)
            "inward_offset": 0.0,           # Must be 0 for direct control
            "payload_offset_x": goto_x,     # Takeoff x = first CSV point x
            "payload_offset_y": goto_y,     # Takeoff y = first CSV point y

            "num_drones": 3,
            "timer_period": 0.02,
            "alt_tol": 0.01,
        }],
    )

    return [fsm_node]


def generate_launch_description() -> LaunchDescription:
    """
    Launch a single drone with FSM node.
    The drone will:
    1. Take off to the first point in the trajectory CSV (using minimum jerk trajectory)
    2. Hover at that position
    """
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value=EnvironmentVariable('DRONE_ID', default_value='0'),
        description='Drone ID'
    )
    traj_base_dir_arg = DeclareLaunchArgument(
        'traj_base_dir',
        default_value='data/3drone_trajectories_new',
        description='Directory containing drone_*_traj_smoothed_100hz.csv files'
    )

    return LaunchDescription([
        drone_id_arg,
        traj_base_dir_arg,
        OpaqueFunction(function=launch_setup),
    ])
