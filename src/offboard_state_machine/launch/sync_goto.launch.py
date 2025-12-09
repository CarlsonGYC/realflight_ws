#!/usr/bin/env python3
"""
Single-drone launch for the new sync GOTO node.
Reads the first CSV point to set takeoff altitude and lets the node own GOTO timing.
"""

from pathlib import Path
import csv
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def read_first_point(base_dir: str, drone_id: str, use_raw: bool):
    """
    Read the first point from the drone's trajectory CSV file.
    """
    suffix = "_traj_raw_20hz.csv" if use_raw else "_traj_smoothed_100hz.csv"
    csv_path = Path(base_dir) / f"drone_{drone_id}{suffix}"

    if not csv_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {csv_path}")

    with csv_path.open("r") as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader, None)  # skip header
        first_row = next(reader, None)
        if not first_row or len(first_row) < 4:
            raise RuntimeError(f"Malformed first row in {csv_path}")

        x = float(first_row[1])
        y = float(first_row[2])
        z = float(first_row[3])
        return x, y, z


def launch_setup(context):
    drone_id_val = LaunchConfiguration("drone_id").perform(context)
    drone_id_int = int(drone_id_val)
    base_dir = LaunchConfiguration("traj_base_dir").perform(context)
    use_raw = LaunchConfiguration("use_raw_traj").perform(context).lower() in ["true", "1", "yes"]

    goto_x, goto_y, goto_z = read_first_point(base_dir, drone_id_val, use_raw)
    takeoff_altitude = 0.4  # NED: up is negative z

    node = Node(
        package="offboard_state_machine",
        executable="offboard_sync_goto_node",
        name=f"offboard_sync_goto_{drone_id_val}",
        output="screen",
        parameters=[{
            "drone_id": drone_id_int,
            "takeoff_alt": takeoff_altitude,
            "goto_x": goto_x,
            "goto_y": goto_y,
            "goto_z": goto_z,
            "takeoff_time": LaunchConfiguration("takeoff_time"),
            "goto_time": LaunchConfiguration("goto_time"),
            "traj_base_dir": base_dir,
            "use_raw_traj": use_raw,
        }],
        remappings=[
            (f"/state/command_drone_{drone_id_int}", f"/state/command_drone_{drone_id_int}"),
            (f"/state/state_drone_{drone_id_int}", f"/state/state_drone_{drone_id_int}"),
        ],
    )

    return [node]


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value=EnvironmentVariable("DRONE_ID", default_value="0"),
        description="Drone ID (one launch instance per drone)"
    )
    traj_base_dir_arg = DeclareLaunchArgument(
        "traj_base_dir",
        default_value="data/3drone_trajectories_new",
        description="Directory containing drone_*_traj_*.csv files"
    )
    use_raw_traj_arg = DeclareLaunchArgument(
        "use_raw_traj",
        default_value="false",
        description="Use *_traj_raw_20hz.csv instead of *_traj_smoothed_100hz.csv"
    )
    takeoff_time_arg = DeclareLaunchArgument(
        "takeoff_time",
        default_value="5.0",
        description="Takeoff duration in seconds"
    )
    goto_time_arg = DeclareLaunchArgument(
        "goto_time",
        default_value="10.0",
        description="Goto duration in seconds"
    )

    return LaunchDescription([
        drone_id_arg,
        traj_base_dir_arg,
        use_raw_traj_arg,
        takeoff_time_arg,
        goto_time_arg,
        OpaqueFunction(function=launch_setup),
    ])
