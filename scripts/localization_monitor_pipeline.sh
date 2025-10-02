#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="ais_localization_pipeline"
PIPELINE_WINDOW="pipeline"
RVIZ_WINDOW="rviz"
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

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required to run this script." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI is not available in the current environment." >&2
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "A tmux session named ${SESSION_NAME} is already running. Attach to it with 'tmux attach -t ${SESSION_NAME}'." >&2
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

# Start tmux session with the pipeline launch
if [ ${#LAUNCH_ARGS[@]} -gt 0 ]; then
  tmux new-session -d -s "${SESSION_NAME}" -n "${PIPELINE_WINDOW}" "ros2 launch robot_localization localization_monitor.launch.py ${LAUNCH_ARGS[*]}"
else
  tmux new-session -d -s "${SESSION_NAME}" -n "${PIPELINE_WINDOW}" "ros2 launch robot_localization localization_monitor.launch.py"
fi

tmux set-option -t "${SESSION_NAME}" remain-on-exit on

if [ "${START_RVIZ}" = true ] && [ -f "${rviz_config}" ]; then
  tmux new-window -t "${SESSION_NAME}" -n "${RVIZ_WINDOW}" "rviz2 -d ${rviz_config}"
fi

tmux select-window -t "${SESSION_NAME}:${PIPELINE_WINDOW}"
tmux attach -t "${SESSION_NAME}"
