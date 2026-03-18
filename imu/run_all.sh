#!/bin/bash
set -euo pipefail

# One-key runner: launch bringup in a new terminal (Terminal A) and imu_turn180 in current terminal (Terminal B).
# Requirements: gnome-terminal (preferred) or xterm available in PATH.
# Usage: ./run_all.sh

WS_ROOT="$(cd "$(dirname "$0")" && pwd)"
BRINGUP_CMD="cd $WS_ROOT/.. && roslaunch upros_bringup bringup_w2a.launch"
TURN_CMD="cd $WS_ROOT && ./start.sh"

launch_new_terminal() {
  local title="$1"; shift
  local cmd="$*"
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -c "$cmd; echo; echo '[exit] close this window if not needed'; exec bash" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd; echo; echo '[exit] close this window if not needed'; read -p 'Enter to close'" &
  else
    echo "[run_all] ERROR: no gnome-terminal or xterm found. Please run bringup manually: $cmd" >&2
    exit 1
  fi
}

# Launch bringup in new terminal (Terminal A)
echo "[run_all] Launching bringup in new terminal..."
launch_new_terminal "Terminal A - bringup" "$BRINGUP_CMD"

# Wait a bit for bringup to start
sleep 3

# Run imu_turn180 in current terminal (Terminal B)
echo "[run_all] Running imu_turn180 in this terminal (Terminal B)..."
exec bash -lc "$TURN_CMD"
