#!/bin/bash
set -euo pipefail

# Unified one-key launcher for imu_turn180
# Features:
#  - Source system ROS (Noetic) if available
#  - Require external bringup (roslaunch upros_bringup bringup_w2a.launch in another terminal)
#  - Initialize catkin workspace if needed, build if devel/setup.bash missing or --rebuild
#  - Optionally launch mock IMU (static yaw or cmd_vel-driven)
#  - Launch imu_turn180_node (foreground by default; --bg for background)

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG=imu_turn180
NODE=imu_turn180_node
SYS_ROS_SETUP="/opt/ros/noetic/setup.bash"

# Defaults: real robot path (no mock); enable mock via --mock
USE_MOCK=false
MOCK_LISTEN_CMD=true
MOCK_YAW_DEG=0
MOCK_RATE=50
BACKGROUND=false
REBUILD=false
EXTRA_ARGS=()

# Environment checks
CHECK_ENV=true

# Default node args (used when no ROS args are provided)
DEFAULT_TARGET_DEG=180
DEFAULT_ANGULAR_SPEED=0.8
DEFAULT_MIN_ANGULAR_SPEED=0.3
DEFAULT_TIMEOUT_S=12.0
DEFAULT_KP=1.6
DEFAULT_BRAKE_TIME=0.3

usage() {
  cat <<'EOF'
Usage: ./start.sh [options] [-- ROS_ARGS]
Note: 请先在另一终端运行底盘/IMU bringup：
  roslaunch upros_bringup bringup_w2a.launch

Options:
  --no-mock            Disable mock IMU (default behavior)
  --mock               Enable mock IMU
  --mock-static        Mock IMU does NOT listen to /cmd_vel (keeps yaw fixed)
  --mock-listen-cmd    Mock IMU listens to /cmd_vel angular.z (default on)
  --mock-yaw DEG       Initial yaw deg for mock IMU (default 0)
  --mock-rate HZ       Publish rate for mock IMU (default 50)
  --bg                 Run imu_turn180_node in background (logs -> imu_turn180_node.log)
  --rebuild            Force catkin_make even if devel/setup.bash exists
  --check-env          Run environment checks (default on)
  --no-check-env       Skip environment checks
  -h, --help           Show this help
  --                   Everything after this is passed to imu_turn180_node as ROS args
Defaults (when no ROS args are provided):
  _target_deg:=180 _angular_speed:=0.8 _min_angular_speed:=0.3
  _timeout_s:=12.0 _kp:=1.6 _brake_time:=0.3
EOF
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
  --mock) USE_MOCK=true; shift;;
  --no-mock) USE_MOCK=false; shift;;
  --mock-listen-cmd) MOCK_LISTEN_CMD=true; shift;;
  --mock-static) MOCK_LISTEN_CMD=false; shift;;
    --mock-yaw) MOCK_YAW_DEG="$2"; shift 2;;
    --mock-rate) MOCK_RATE="$2"; shift 2;;
    --bg) BACKGROUND=true; shift;;
    --rebuild) REBUILD=true; shift;;
    --check-env) CHECK_ENV=true; shift;;
    --no-check-env) CHECK_ENV=false; shift;;
    -h|--help) usage; exit 0;;
    --) shift; EXTRA_ARGS+=("$@" ); break;;
    *) EXTRA_ARGS+=("$1"); shift;;
  esac
done

echo "[start] workspace: $WS_DIR"

# Source system ROS
if [ -f "$SYS_ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$SYS_ROS_SETUP"
else
  echo "[start] Warning: system ROS setup not found at $SYS_ROS_SETUP"
fi

ensure_master() {
  if ! rosnode list >/dev/null 2>&1; then
    echo "[start] ERROR: ROS master 未运行。请先在另一终端执行: roslaunch upros_bringup bringup_w2a.launch" >&2
    exit 1
  fi
}

  check_imu_pub() {
    local topic="$1"
    local n
    n=$(rostopic info "$topic" 2>/dev/null | awk '/Publishers:/{flag=1;next}/Subscribers:/{flag=0}flag && NF{print}' | wc -l)
    if [ "$n" -eq 0 ]; then
      echo "[start][check] No publishers on $topic" >&2
      return 1
    fi
    echo "[start][check] $topic publishers: $n"
    return 0
  }

  check_cmd_vel_sub() {
    local topic="$1"
    local n
    n=$(rostopic info "$topic" 2>/dev/null | awk '/Subscribers:/{flag=1;next}/Publishers:/{flag=0}flag && NF{print}' | wc -l)
    if [ "$n" -eq 0 ]; then
      echo "[start][check] No subscribers on $topic" >&2
      return 1
    fi
    echo "[start][check] $topic subscribers: $n"
    return 0
  }

  check_zoo_driver() {
    if ! rosnode list 2>/dev/null | grep -q "/zoo_driver$"; then
      echo "[start][check] zoo_driver 未检测到。请在另一终端运行: roslaunch upros_bringup bringup_w2a.launch" >&2
      return 1
    fi
    echo "[start][check] zoo_driver running"
    return 0
  }

ensure_master

# Initialize catkin workspace if needed
if [ ! -f "$WS_DIR/src/CMakeLists.txt" ]; then
  echo "[start] initializing catkin workspace"
  (cd "$WS_DIR/src" && catkin_init_workspace)
fi

# Build if needed or forced
if [ "$REBUILD" = true ] || [ ! -f "$WS_DIR/devel/setup.bash" ]; then
  echo "[start] catkin_make ..."
  (cd "$WS_DIR" && catkin_make)
else
  echo "[start] workspace already built (devel/setup.bash present)"
fi

# Source workspace
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"

# Check package visibility
if ! rospack find "$PKG" >/dev/null 2>&1; then
  echo "[start] ERROR: package '$PKG' not found after sourcing."
  exit 2
fi

# Environment checks
if [ "$CHECK_ENV" = true ]; then
  # IMU publisher: skip if using mock (mock will publish)
  if [ "$USE_MOCK" = false ]; then
    check_zoo_driver || { echo "[start] 需要先启动底盘驱动: roslaunch upros_bringup bringup_w2a.launch"; exit 3; }
    check_imu_pub /imu/data || { echo "[start] IMU not detected on /imu/data. Use --no-check-env to skip."; exit 3; }
  fi
  # cmd_vel subscriber: only meaningful for real robot (no mock)
  check_cmd_vel_sub /cmd_vel || echo "[start] Warning: no subscribers on /cmd_vel (is base driver running?)"
fi

MOCK_PID=""
if [ "$USE_MOCK" = true ]; then
  MOCK_CMD=(rosrun "$PKG" mock_imu_pub.py _topic:=/imu/data _rate:=$MOCK_RATE _yaw_deg:=$MOCK_YAW_DEG)
  if [ "$MOCK_LISTEN_CMD" = true ]; then
    MOCK_CMD+=(_listen_cmd_vel:=true _cmd_vel_topic:=/cmd_vel)
  fi
  echo "[start] launching mock IMU (${MOCK_CMD[*]})"
  nohup "${MOCK_CMD[@]}" > "$WS_DIR/mock_imu.log" 2>&1 &
  MOCK_PID=$!
  echo "[start] mock IMU pid=$MOCK_PID (logs -> $WS_DIR/mock_imu.log)"
fi

NODE_CMD=(rosrun "$PKG" "$NODE")
if [ ${#EXTRA_ARGS[@]} -eq 0 ]; then
  EXTRA_ARGS=(
    _target_deg:=${DEFAULT_TARGET_DEG}
    _angular_speed:=${DEFAULT_ANGULAR_SPEED}
    _min_angular_speed:=${DEFAULT_MIN_ANGULAR_SPEED}
    _timeout_s:=${DEFAULT_TIMEOUT_S}
    _kp:=${DEFAULT_KP}
    _brake_time:=${DEFAULT_BRAKE_TIME}
  )
fi
NODE_CMD+=("${EXTRA_ARGS[@]}")

if [ "$BACKGROUND" = true ]; then
  echo "[start] launching node in background: ${NODE_CMD[*]} (logs -> $WS_DIR/${NODE}.log)"
  nohup "${NODE_CMD[@]}" > "$WS_DIR/${NODE}.log" 2>&1 &
  echo "[start] node pid=$!"
  if [ -n "$MOCK_PID" ]; then echo "[start] mock pid=$MOCK_PID"; fi
  exit 0
else
  echo "[start] launching node in foreground: ${NODE_CMD[*]}"
  # Clean up mock on exit
  trap 'if [ -n "$MOCK_PID" ]; then kill $MOCK_PID 2>/dev/null || true; fi' EXIT
  exec "${NODE_CMD[@]}"
fi
