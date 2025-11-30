#!/bin/bash

SESSION_NAME="multi_uav_session"

# Kill old session if exists (optional)
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session with first command
tmux new-session -d -s $SESSION_NAME

# Pane 1: traj_test (trajectory server)
tmux send-keys -t $SESSION_NAME "source ./install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "ros2 launch traj_test multi_follow.launch.py" C-m

# Split into second pane
tmux split-window -h -t $SESSION_NAME

# Wait 5 seconds before running the second command
tmux send-keys -t $SESSION_NAME.1 "sleep 5" C-m
tmux send-keys -t $SESSION_NAME.1 "source ./install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME.1 "ros2 launch offboard_state_machine swarm_follow.launch.py" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME
