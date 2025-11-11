#!/usr/bin/env python3
"""
Single drone launch file for offboard FSM
Launches one drone that takes off to 1.5m and hovers at the takeoff position
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch a single drone with FSM node.
    The drone will:
    1. Take off to 1.5m above current position (0, 0, -1.5 in NED)
    2. Hover at the takeoff position (goto target same as takeoff target)
    """
    
    # Single drone configuration
    drone_id = 0
    takeoff_altitude = 1.5  # meters above ground
    
    # In NED frame: x=North, y=East, z=Down
    # Takeoff position: hover at origin with 1.2m altitude
    goto_x = - 1.50   
    goto_y = 0.0    
    goto_z = -takeoff_altitude  # NED: negative means up
    
    fsm_node = Node(
        package="offboard_state_machine",
        executable="offboard_fsm_node",
        name=f"offboard_fsm_node_{drone_id}",
        output="screen",
        parameters=[{
            "drone_id": drone_id,
            "takeoff_alt": takeoff_altitude,
            "takeoff_time": 3.0,        # seconds to reach takeoff altitude
            "climb_rate": 1.0,          # m/s (will be overridden by takeoff_alt/takeoff_time)
            "landing_time": 2.0,        # seconds for landing
            'goto_x': goto_x,             # GOTO 
            'goto_y': goto_y,             
            'goto_z': goto_z,            
            'goto_tol': 0.05,           # meters
            'goto_max_vel': 1.0,        # m/s
            'goto_accel_time': 4.0,    # seconds to reach max velocity
            "num_drones": 1,            # single drone
            "timer_period": 0.02,       # 50 Hz control loop
            "alt_tol": 0.01,            # altitude tolerance in meters
            "inward_offset": 0.0,       # no offset for single drone at origin
            "payload_offset_x": 0.0,    # no payload offset
            "payload_offset_y": 0.00   # match goto_y for hover
        }],
    )

    return LaunchDescription([
        fsm_node
    ])