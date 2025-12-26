#!/usr/bin/env bash
set -euo pipefail

# Launch multi-drone geom_multilift HITL + sync goto with bag recording.
# Requires a trajectory directory argument that is forwarded to geom_multilift params
# and the sync_goto state machine.

DRONE_ID="${DRONE_ID:-0}"
SESSION_NAME="hitl_uav_session_${DRONE_ID}"
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

PARAMS_TEMPLATE="$WS_DIR/src/geom_multilift/config/geom_multilift_hitl_params.yaml"
if [[ ! -f "$PARAMS_TEMPLATE" ]]; then
  echo "geom_multilift HITL params template not found: $PARAMS_TEMPLATE" >&2
  exit 1
fi

PARAMS_DIR="$WS_DIR/tmp"
mkdir -p "$PARAMS_DIR"
PARAMS_FILE="${PARAMS_DIR}/geom_multilift_hitl_params_${DRONE_ID}_$(date +%Y%m%d_%H%M%S).yaml"
sed "s#^[[:space:]]*data_root:.*#    data_root: \"${TRAJ_DIR}\"#" "$PARAMS_TEMPLATE" > "$PARAMS_FILE"
PARAMS_FILE_ESCAPED="$(printf '%q' "$PARAMS_FILE")"

echo "Using geom_multilift data root: $TRAJ_DIR"
echo "Using params file: $PARAMS_FILE"

# Source ROS without nounset to avoid unbound variable issues from upstream scripts
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

if [[ "$DRONE_ID" -eq 0 ]]; then
  PX4_NAMESPACE="/fmu/"
else
  PX4_NAMESPACE="/px4_${DRONE_ID}/fmu/"
fi

STATE_CMD_TOPIC="/state/command_drone_${DRONE_ID}"
STATE_STATE_TOPIC="/state/state_drone_${DRONE_ID}"
TRAJ_TOPIC="${PX4_NAMESPACE}in/trajectory_setpoint"
LOCAL_POS_TOPIC="${PX4_NAMESPACE}out/vehicle_local_position"

start_tmux_with_recording() {
  # Kill old session for this drone
  tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

  # Create session and split layout: LEFT (geom_multilift), RIGHT-TOP (sync_goto), RIGHT-BOTTOM (rosbag)
  tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"
  tmux split-window -h -t "$SESSION_NAME" -c "$WS_DIR"
  readarray -t PANES < <(tmux list-panes -t "$SESSION_NAME" -F '#{pane_id}')
  LEFT_PANE="${PANES[0]}"
  RIGHT_PANE="${PANES[1]}"
  RECORD_PANE=$(tmux split-window -v -t "$RIGHT_PANE" -c "$WS_DIR" -P -F '#{pane_id}')

  # LEFT: geom_multilift HITL + swarm coordinator (launch includes coordinator)
  tmux send-keys -t "$LEFT_PANE" "cd $WS_DIR" C-m
  tmux send-keys -t "$LEFT_PANE" "source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
  tmux send-keys -t "$LEFT_PANE" "ros2 launch geom_multilift geom_multilift_hitl_sync.launch.py params_file:=${PARAMS_FILE_ESCAPED}" C-m

  # RIGHT: sync_goto (slight delay to let geom_multilift come up) with same trajectory directory
  tmux send-keys -t "$RIGHT_PANE" "cd $WS_DIR" C-m
  tmux send-keys -t "$RIGHT_PANE" "sleep 5 && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
  tmux send-keys -t "$RIGHT_PANE" "ros2 launch offboard_state_machine sync_goto.launch.py traj_base_dir:=${TRAJ_DIR_ESCAPED}" C-m

  # RECORD: rosbag
  BAG_DIR="$WS_DIR/rosbags"
  mkdir -p "$BAG_DIR"
  BAG_PATH="${BAG_DIR}/drone_${DRONE_ID}_$(date +%Y%m%d_%H%M%S)"

  tmux send-keys -t "$RECORD_PANE" "cd $WS_DIR" C-m
  tmux send-keys -t "$RECORD_PANE" "source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" C-m
  tmux send-keys -t "$RECORD_PANE" "echo \"Recording ROS 2 bag for drone ${DRONE_ID}\"" C-m
  tmux send-keys -t "$RECORD_PANE" "echo \"Topics:\"; echo \"  - ${STATE_CMD_TOPIC}\"; echo \"  - ${STATE_STATE_TOPIC}\"; echo \"  - ${TRAJ_TOPIC}\"; echo \"  - ${LOCAL_POS_TOPIC}\"" C-m
  tmux send-keys -t "$RECORD_PANE" "echo \"Output: ${BAG_PATH}\"" C-m
  tmux send-keys -t "$RECORD_PANE" "ros2 bag record -o ${BAG_PATH} ${STATE_CMD_TOPIC} ${STATE_STATE_TOPIC} ${TRAJ_TOPIC} ${LOCAL_POS_TOPIC}" C-m

  tmux attach-session -t "$SESSION_NAME"
}

start_tmux_with_recording
