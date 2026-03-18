#!/bin/bash
set -e
# One-shot build+run helper for imu_turn180 inside this imu/ folder
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_NAME=imu_turn180

echo "Workspace dir: $WS_DIR"
# ensure src exists
mkdir -p "$WS_DIR/src"

# If package still lives in parent workspace src (old location), move it here
if [ -d "$WS_DIR/../src/$PKG_NAME" ] && [ ! -d "$WS_DIR/src/$PKG_NAME" ]; then
  echo "Moving package from parent workspace into this workspace..."
  mv "$WS_DIR/../src/$PKG_NAME" "$WS_DIR/src/"
fi

# If package not present, show error
if [ ! -d "$WS_DIR/src/$PKG_NAME" ]; then
  echo "ERROR: package $PKG_NAME not found in $WS_DIR/src"
  exit 2
fi

# Initialize catkin workspace if needed
if [ ! -f "$WS_DIR/src/CMakeLists.txt" ]; then
  echo "Initializing catkin workspace..."
  (cd "$WS_DIR/src" && catkin_init_workspace)
fi

# Build
echo "Building workspace (catkin_make)..."
cd "$WS_DIR"
catkin_make

# Source and run
echo "Sourcing devel/setup.bash"
source "$WS_DIR/devel/setup.bash"

# Pass through all args to the node
echo "Running node imu_turn180_node (forwarding args: $@)"
rosrun $PKG_NAME imu_turn180_node "$@"
