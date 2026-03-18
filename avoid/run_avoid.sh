#!/bin/bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
SYS_ROS_SETUP="/opt/ros/noetic/setup.bash"

if [ -f "$SYS_ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$SYS_ROS_SETUP"
else
  echo "[run_avoid] system ROS not found at $SYS_ROS_SETUP" >&2
fi

if [ ! -f "$WS_DIR/devel/setup.bash" ]; then
  echo "[run_avoid] devel/setup.bash not found, building..."
  (cd "$WS_DIR" && catkin_make)
fi
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"

if ! rosnode list >/dev/null 2>&1; then
  echo "[run_avoid] ERROR: ROS master 未运行。请先在另一终端执行: roslaunch upros_bringup bringup_w2a.launch" >&2
  exit 1
fi

CMD=(roslaunch obstacle_avoid obstacle_avoid.launch)
if [ "$#" -gt 0 ]; then
  CMD+=("$@")
fi

echo "[run_avoid] Running: ${CMD[*]}"
exec "${CMD[@]}"
