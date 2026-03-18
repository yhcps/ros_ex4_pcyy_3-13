#!/bin/bash
set -euo pipefail

# One-key: open new terminal for bringup, current terminal runs obstacle_avoid

WS_ROOT="$(cd "$(dirname "$0")" && pwd)"
# 默认纯触碰调试：关闭超声/TOF，可通过环境变量 BRINGUP_ARGS 覆盖
BRINGUP_ARGS=${BRINGUP_ARGS:-"enable_ultra:=false enable_tof:=false"}
BRINGUP_CMD="cd $WS_ROOT/.. && roslaunch upros_bringup bringup_w2a.launch $BRINGUP_ARGS"
AVOID_CMD="cd $WS_ROOT && ./run_avoid.sh"

launch_new_terminal() {
  local title="$1"; shift
  local cmd="$*"
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -c "$cmd; echo; echo '[exit] close this window if not needed'; exec bash" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd; echo; echo '[exit] close this window if not needed'; read -p 'Enter to close'" &
  else
    echo "[run_all_avoid] ERROR: no gnome-terminal or xterm found. Please run bringup manually: $cmd" >&2
    exit 1
  fi
}

echo "[run_all_avoid] Launching bringup in new terminal..."
launch_new_terminal "Terminal A - bringup" "$BRINGUP_CMD"

sleep 3

echo "[run_all_avoid] Running obstacle_avoid in this terminal..."
exec bash -lc "$AVOID_CMD"
