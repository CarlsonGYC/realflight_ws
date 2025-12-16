#!/usr/bin/env bash
set -euo pipefail

# Single-drone launcher for follow_traj + swarm_follow with a required trajectory directory.
# The provided path is forwarded to both launch files so the C++ nodes load the correct CSV.

DRONE_ID="${DRONE_ID:-0}"
SESSION_NAME="single_uav_session_${DRONE_ID}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <trajectory_directory>"
  exit 1
fi

resolve_traj_dir() {
  local input="$1"
  if [[ -d "$input" ]]; then
    realpath "$input"
    return
  fi
  if [[ -d "$WS_DIR/$input" ]]; then
    realpath "$WS_DIR/$input"
    return
  fi
  echo "Trajectory directory not found: $input" >&2
  exit 1
}

TRAJ_DIR="$(resolve_traj_dir "$1")"
TRAJ_DIR_ESCAPED="$(printf '%q' "$TRAJ_DIR")"

echo "Using trajectory directory: $TRAJ_DIR"

# Source ROS without nounset to avoid unbound variable issues from upstream scripts
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

# Kill old session for this drone
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session and split layout: LEFT (follow_traj), RIGHT (swarm_follow)
tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"
tmux split-window -h -t "$SESSION_NAME" -c "$WS_DIR"
readarray -t PANES < <(tmux list-panes -t "$SESSION_NAME" -F '#{pane_id}')
LEFT_PANE="${PANES[0]}"
RIGHT_PANE="${PANES[1]}"

# LEFT: follow_traj with CSV base path forwarded
tmux send-keys -t "$LEFT_PANE" "cd $WS_DIR" C-m
tmux send-keys -t "$LEFT_PANE" "source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
tmux send-keys -t "$LEFT_PANE" "ros2 launch traj_test follow_traj.launch.py csv_base_path:=${TRAJ_DIR_ESCAPED}" C-m

# RIGHT: swarm_follow with matching trajectory directory (slight delay)
tmux send-keys -t "$RIGHT_PANE" "cd $WS_DIR" C-m
tmux send-keys -t "$RIGHT_PANE" "sleep 3 && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
tmux send-keys -t "$RIGHT_PANE" "ros2 launch offboard_state_machine swarm_follow.launch.py traj_base_dir:=${TRAJ_DIR_ESCAPED}" C-m

tmux attach-session -t "$SESSION_NAME"
