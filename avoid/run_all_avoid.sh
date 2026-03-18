#!/bin/bash
set -euo pipefail

# One-key: 在当前调用中先启动 bringup（启用超声+TOF+触碰），再运行 obstacle_avoid

WS_ROOT="$(cd "$(dirname "$0")" && pwd)"

BRINGUP_CMD=(roslaunch upros_bringup bringup_w2a.launch enable_ultra:=true enable_tof:=true)
AVOID_CMD=(bash -lc "cd '$WS_ROOT' && ./run_avoid.sh")

BRINGUP_PID=""

cleanup() {
  if [[ -n "$BRINGUP_PID" ]] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
    echo "[run_all_avoid] Stopping bringup (PID=$BRINGUP_PID)..."
    kill "$BRINGUP_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[run_all_avoid] Starting bringup (ultra+tof on)..."
(cd "$WS_ROOT/.." && "${BRINGUP_CMD[@]}") &
BRINGUP_PID=$!
sleep 3

echo "[run_all_avoid] Running obstacle_avoid..."
exec "${AVOID_CMD[@]}"
