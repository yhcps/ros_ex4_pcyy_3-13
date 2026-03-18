#!/bin/bash
set -euo pipefail

# One-key bringup: 启用超声 + TOF + 触碰，简化参数。

WS_ROOT="$(cd "$(dirname "$0")" && pwd)"
CMD=(roslaunch upros_bringup bringup_w2a.launch enable_ultra:=true enable_tof:=true)

echo "[bringup_touch] Running: ${CMD[*]}"
exec "${CMD[@]}"
