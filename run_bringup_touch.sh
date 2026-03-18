#!/bin/bash
set -euo pipefail

# One-key bringup for touch-only test: disable ultrasonic and TOF by default.
# Override via BRINGUP_ARGS env if needed, e.g. BRINGUP_ARGS="enable_ultra:=true enable_tof:=false" ./run_bringup_touch.sh

WS_ROOT="$(cd "$(dirname "$0")" && pwd)"
BRINGUP_ARGS=${BRINGUP_ARGS:-"enable_ultra:=false enable_tof:=false"}
CMD=(roslaunch upros_bringup bringup_w2a.launch $BRINGUP_ARGS)

echo "[bringup_touch] Running: ${CMD[*]}"
exec "${CMD[@]}"
