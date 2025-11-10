#!/bin/bash
# filepath: /home/tlab-uav/realflight_ws/start_system.sh

# ============================================================================
# UAV System Startup Script with Screen Sessions
# ============================================================================
# This script starts multiple ROS2 modules in separate screen sessions for
# background operation. Each module runs independently and can be accessed
# via screen commands.
#
# Required Environment Variables:
#   - GCS_IP: Ground Control Station IP address for QGroundControl
#   - DRONE_NAME_VICON: Vicon object name for the drone
#   - VICON_IP: IP address of the Vicon motion capture server
#
# Screen Sessions Created:
#   1. px4_microdds: PX4 MicroXRCE-DDS Agent for serial communication
#   2. qgc_forward: Serial-to-UDP forwarder for QGroundControl
#   3. vicon_client: VRPN/Vicon motion capture client
#   4. vicon_bridge: Vicon to PX4 data bridge
# ============================================================================

set -e  # Exit on any error

# ============================================================================
# Environment Variable Validation
# ============================================================================

echo "======================================"
echo "sync with remote time server if ssh connected..."
echo "======================================"

if [[ -n "$SSH_CONNECTION" ]]; then
    REMOTE_IP=$(echo $SSH_CONNECTION | awk '{print $1}')
    if command -v ntpdate >/dev/null 2>&1; then
        sudo ntpdate -u "$REMOTE_IP" && echo "time already sync with $REMOTE_IP."
    fi
fi

echo "======================================"
echo "Checking Environment Variables..."
echo "======================================"

# Check if required environment variables are set
if [ -z "$GCS_IP" ]; then
    echo "ERROR: GCS_IP environment variable is not set"
    echo "Please set it with: export GCS_IP=<ground_station_ip>"
    exit 1
fi

if [ -z "$DRONE_NAME_VICON" ]; then
    echo "ERROR: DRONE_NAME_VICON environment variable is not set"
    echo "Please set it with: export DRONE_NAME_VICON=<vicon_object_name>"
    exit 1
fi

if [ -z "$VICON_IP" ]; then
    echo "ERROR: VICON_IP environment variable is not set"
    echo "Please set it with: export VICON_IP=<vicon_server_ip>"
    exit 1
fi

# Display current environment configuration
echo "GCS_IP: $GCS_IP"
echo "DRONE_NAME_VICON: $DRONE_NAME_VICON"
echo "VICON_IP: $VICON_IP"
echo "======================================"
echo ""

# ============================================================================
# ROS2 Environment Setup Command
# ============================================================================
# This command will be executed at the start of each screen session to
# properly configure the ROS2 environment
WORKSPACE_DIR="$HOME/realflight_ws"
ROS2_SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# ============================================================================
# Function: Start Screen Session
# ============================================================================
# Usage: start_screen_session <session_name> <command>
# Creates a detached screen session with ROS2 environment sourced
start_screen_session() {
    local session_name=$1
    local command=$2
    
    echo "Starting screen session: $session_name"
    
    # Check if screen session already exists
    if screen -list | grep -q "$session_name"; then
        echo "  WARNING: Screen session '$session_name' already exists. Killing it..."
        screen -S "$session_name" -X quit
        sleep 1
    fi
    
    # Create new detached screen session with proper environment setup
    screen -dmS "$session_name" bash -c "$ROS2_SETUP_CMD && $command; exec bash"
    echo "  ✓ Screen session '$session_name' started"
    sleep 1
}

# ============================================================================
# Module 1: PX4 MicroXRCE-DDS Agent
# ============================================================================
# Establishes communication between PX4 autopilot and ROS2 via serial port
# - Device: /dev/ttyAML1 (PX4 serial connection)
# - Baud Rate: 921600
# - Protocol: MicroXRCE-DDS for efficient ROS2 communication
echo ""
echo "======================================"
echo "Starting PX4 MicroXRCE-DDS Agent..."
echo "======================================"
PX4_MICRODDS_CMD="MicroXRCEAgent serial --dev /dev/ttyAML1 -b 921600"
start_screen_session "px4_microdds" "$PX4_MICRODDS_CMD"

# ============================================================================
# Module 2: QGroundControl Serial-to-UDP Forwarder
# ============================================================================
# Forwards MAVLink messages from PX4 to QGroundControl via UDP
# - Input: /dev/ttyACM0 (PX4 serial port for telemetry)
# - Baud Rate: 115200
# - Output: UDP to GCS_IP:14550 (QGC default port)
echo ""
echo "======================================"
echo "Starting QGroundControl Forwarder..."
echo "======================================"
QGC_FORWARD_CMD="socat -d -d /dev/ttyACM0,raw,b115200,echo=0 UDP-SENDTO:$GCS_IP:14550"
start_screen_session "qgc_forward" "$QGC_FORWARD_CMD"

# ============================================================================
# Module 3: VRPN/Vicon Motion Capture Client
# ============================================================================
# Connects to Vicon motion capture system and publishes pose data to ROS2
# - Server: VICON_IP (Vicon Tracker server address)
# - Port: 3883 (VRPN default port)
# - Publishes: Transform data for tracked objects
echo ""
echo "======================================"
echo "Starting VRPN/Vicon Client..."
echo "======================================"
VICON_CLIENT_CMD="ros2 launch vrpn_mocap client.launch.yaml server:=$VICON_IP port:=3883"
start_screen_session "vicon_client" "$VICON_CLIENT_CMD"

# ============================================================================
# Module 4: Vicon to PX4 Bridge
# ============================================================================
# Converts Vicon pose data to PX4-compatible format and publishes
# - Reads: Vicon pose from vrpn_mocap
# - Publishes: Vision-based position estimate to PX4
# - Handles: Frame transformations (ENU/NED/FLU conversions)
echo ""
echo "======================================"
echo "Starting Vicon-PX4 Bridge..."
echo "======================================"
VICON_BRIDGE_CMD="ros2 launch vicon_px4_bridge vicon_px4_bridge.launch.py"
start_screen_session "vicon_bridge" "$VICON_BRIDGE_CMD"

# ============================================================================
# Startup Complete
# ============================================================================
echo ""
echo "======================================"
echo "All modules started successfully!"
echo "======================================"
echo ""
echo "Active screen sessions:"
screen -list
echo ""
echo "To attach to a screen session, use:"
echo "  screen -r <session_name>"
echo ""
echo "Available sessions:"
echo "  - px4_microdds: PX4 MicroXRCE-DDS Agent"
echo "  - qgc_forward: QGroundControl Forwarder"
echo "  - vicon_client: Vicon Motion Capture Client"
echo "  - vicon_bridge: Vicon-PX4 Bridge"
echo ""
echo "To detach from a screen session, press: Ctrl+A, then D"
echo "To stop all sessions, run: ./stop_system.sh"
echo "======================================"