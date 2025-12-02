#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="multi_uav_session_$DRONE_ID"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Kill old session
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session and split immediately
tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"
tmux split-window -h -t "$SESSION_NAME" -c "$WS_DIR"

# Use pane IDs instead of numeric indexes so base-index settings don't break targeting
readarray -t PANES < <(tmux list-panes -t "$SESSION_NAME" -F '#{pane_id}')
LEFT_PANE="${PANES[0]}"
RIGHT_PANE="${PANES[1]}"

# Send to LEFT pane
tmux send-keys -t "$LEFT_PANE" "cd $WS_DIR" C-m
tmux send-keys -t "$LEFT_PANE" "source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
tmux send-keys -t "$LEFT_PANE" "ros2 launch traj_test multi_follow.launch.py" C-m

# Send to RIGHT pane
tmux send-keys -t "$RIGHT_PANE" "cd $WS_DIR" C-m
tmux send-keys -t "$RIGHT_PANE" "sleep 5 && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
tmux send-keys -t "$RIGHT_PANE" "ros2 launch offboard_state_machine swarm_follow.launch.py" C-m

# Attach
tmux attach-session -t "$SESSION_NAME"
