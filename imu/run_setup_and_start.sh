#!/bin/bash
set -euo pipefail

# run_setup_and_start.sh
# - Ensure ROS is sourced
# - Start roscore if not running
# - Build (catkin_make) if needed
# - Source workspace devel/setup.bash
# - Run imu_turn180_node (foreground by default; pass --background to run detached)

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG=imu_turn180
NODE=imu_turn180_node
SYS_ROS_SETUP="/opt/ros/noetic/setup.bash"

BACKGROUND=false
# parse optional --background flag
if [ "$#" -gt 0 ] && [ "$1" = "--background" ]; then
  BACKGROUND=true
  shift
fi

echo "[run_setup_and_start] workspace: $WS_DIR"

# source system ROS if available
if [ -f "$SYS_ROS_SETUP" ]; then
  echo "Sourcing system ROS setup: $SYS_ROS_SETUP"
  # shellcheck disable=SC1090
  source "$SYS_ROS_SETUP"
else
  echo "Warning: system ROS setup not found at $SYS_ROS_SETUP"
fi

# Ensure roscore
if pgrep -x roscore >/dev/null 2>&1; then
  echo "roscore already running"
else
  echo "Starting roscore in background... (logs -> $WS_DIR/roscore.log)"
  nohup roscore > "$WS_DIR/roscore.log" 2>&1 &
  # wait for ros master to be available
  echo -n "Waiting for ros master..."
  n=0
  until rosnode list >/dev/null 2>&1; do
    sleep 0.5
    n=$((n+1))
    echo -n "."
    if [ $n -gt 40 ]; then
      echo "\nTimeout waiting for roscore"
      exit 1
    fi
  done
  echo " OK"
fi

# Prepare workspace
if [ ! -f "$WS_DIR/src/CMakeLists.txt" ]; then
  echo "Initializing catkin workspace in $WS_DIR/src"
  (cd "$WS_DIR/src" && catkin_init_workspace) >/dev/null 2>&1 || true
fi

# Build if needed (if devel/setup.bash missing)
if [ ! -f "$WS_DIR/devel/setup.bash" ]; then
  echo "Building workspace (catkin_make)..."
  (cd "$WS_DIR" && catkin_make) || { echo "catkin_make failed"; exit 1; }
else
  echo "Workspace already built (devel/setup.bash exists)"
fi

# source workspace
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"

# ensure package exists
if ! rospack find "$PKG" >/dev/null 2>&1; then
  echo "ERROR: package '$PKG' not found after sourcing. Check workspace and package location."
  exit 2
fi

# Run node
if [ "$BACKGROUND" = true ]; then
  echo "Launching $PKG/$NODE in background (logs -> $WS_DIR/${NODE}.log)"
  nohup rosrun "$PKG" "$NODE" "$@" > "$WS_DIR/${NODE}.log" 2>&1 &
  echo "Node started (pid $!)"
else
  echo "Launching $PKG/$NODE in foreground"
  exec rosrun "$PKG" "$NODE" "$@"
fi
