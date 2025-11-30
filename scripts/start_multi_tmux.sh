#!/bin/bash

SESSION_NAME="multi_uav_session"
WS_DIR="$PWD"   # or set to your workspace path explicitly

# Kill old session if exists (optional)
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# Create new session, window 0, pane 0, and start in workspace dir
tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"

# Pane 0: traj_test
tmux send-keys -t "$SESSION_NAME:0.0" \
  "source ./install/setup.bash && ros2 launch traj_test multi_follow.launch.py" C-m

# Split window horizontally to create pane 1
tmux split-window -h -t "$SESSION_NAME:0.0"

# Pane 1: offboard_state_machine (start after 5s)
tmux send-keys -t "$SESSION_NAME:0.1" \
  "sleep 5 && source ./install/setup.bash && ros2 launch offboard_state_machine swarm_follow.launch.py" C-m

# Attach or switch depending on whether we're already in tmux
if [ -z "$TMUX" ]; then
    tmux attach-session -t "$SESSION_NAME"
else
    tmux switch-client -t "$SESSION_NAME"
fi
