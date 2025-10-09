#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="ais_localization_pipeline"
PIPELINE_WINDOW="pipeline"
RVIZ_WINDOW="rviz"
CLI_WINDOW="ros2cli"
LOGS_WINDOW="logs"
LAUNCH_ARGS=("$@")
START_RVIZ=true

for arg in "${LAUNCH_ARGS[@]}"; do
  case "${arg}" in
    start_rviz:=false|start_rviz:=False|start_rviz:=FALSE)
      START_RVIZ=false
      ;;
    start_rviz:=true|start_rviz:=True|start_rviz:=TRUE)
      START_RVIZ=true
      ;;
  esac
done

if ! command -v byobu-tmux >/dev/null 2>&1; then
  echo "byobu with tmux backend is required to run this script." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI is not available in the current environment." >&2
  exit 1
fi

if byobu-tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "A byobu session named ${SESSION_NAME} is already running. Attach to it with 'byobu-tmux attach -t ${SESSION_NAME}'." >&2
  exit 1
fi

pkg_prefix=$(ros2 pkg prefix robot_localization)
pkg_share="${pkg_prefix}/share/robot_localization"
rviz_config="${pkg_share}/config/MonitorAnalysis.rviz"

if [ "${START_RVIZ}" = true ]; then
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "rviz2 was not found in PATH. The pipeline will start without visualization." >&2
    START_RVIZ=false
  fi
fi

# ---- Start byobu-tmux session and windows ----
# Window 1: pipeline launch
if [ ${#LAUNCH_ARGS[@]} -gt 0 ]; then
  byobu-tmux new-session -d -s "${SESSION_NAME}" -n "${PIPELINE_WINDOW}" \
    "ros2 launch robot_localization localization_monitor.launch.py ${LAUNCH_ARGS[*]}"
else
  byobu-tmux new-session -d -s "${SESSION_NAME}" -n "${PIPELINE_WINDOW}" \
    "ros2 launch robot_localization localization_monitor.launch.py"
fi

# Maus aktivieren
byobu-tmux set-option -t "${SESSION_NAME}" -g mouse on

# Session bleibt stehen bei Exit
byobu-tmux set-option -t "${SESSION_NAME}" remain-on-exit on

# Window 2: RViz
if [ "${START_RVIZ}" = true ] && [ -f "${rviz_config}" ]; then
  byobu-tmux new-window -t "${SESSION_NAME}" -n "${RVIZ_WINDOW}" \
    "rviz2 -d ${rviz_config}"
fi

# Window 3: ROS2 CLI shell
byobu-tmux new-window -t "${SESSION_NAME}" -n "${CLI_WINDOW}" \
  "bash --rcfile <(echo 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash') -i"

# Window 4: Logs
byobu-tmux new-window -t "${SESSION_NAME}" -n "${LOGS_WINDOW}" "watch -n 2 ros2 node list"

# Select main window and attach
byobu-tmux select-window -t "${SESSION_NAME}:${PIPELINE_WINDOW}"
byobu-tmux attach -t "${SESSION_NAME}"
