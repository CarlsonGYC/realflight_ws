#!/bin/bash
# filepath: /home/tlab-uav/realflight_ws/stop_system.sh

# ============================================================================
# UAV System Shutdown Script
# ============================================================================
# This script stops all screen sessions created by start_system.sh
# ============================================================================

echo "======================================"
echo "Stopping all UAV system modules..."
echo "======================================"

# List of screen sessions to stop
SESSIONS=("px4_microdds" "qgc_forward" "vicon_client" "vicon_bridge")

for session in "${SESSIONS[@]}"; do
    if screen -list | grep -q "$session"; then
        echo "Stopping screen session: $session"
        screen -S "$session" -X quit
        echo "  ✓ $session stopped"
    else
        echo "  ⓘ $session not running"
    fi
done

echo ""
echo "======================================"
echo "All modules stopped"
echo "======================================"