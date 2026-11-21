#!/usr/bin/env bash
# 重新生成场地 + 无人机（在包根目录生成 rmuc_2025_field.xml / rmuc_2025_drone.xml）
#
#   ./build/build.sh                 # 只重建场地
#   ./build/build.sh --drone         # 场地 + 无人机
#   SUN_P=0.3 LAMP_P=1.2 ./build/build.sh
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

VENV_PY="/data/MuJoCo/MuJoCo-drones-gym/.venv/bin/python"
[ -x "$VENV_PY" ] || VENV_PY="python3"

DRONE_FLAG=0
if [ "${1:-}" = "--drone" ]; then DRONE_FLAG=1; shift; fi

WORLD="${WORLD:-$HOME/one_drone/worlds/rmuc_2025_3m_vio_columns.sdf}"
SEARCH_DIR="${SEARCH_DIR:-$HOME/one_drone/worlds/models}"

"$VENV_PY" build/sdf2mjcf.py "$WORLD" \
    -o rmuc_2025_field.xml \
    --search "$SEARCH_DIR" \
    --exclude wall_segment --exclude my_wall_1 --exclude target_marker \
    --texture assets/field_floor.png \
    --checker-ground "${CHECKER:-1.0}" \
    --dark-sky \
    --center-light "${LAMP_H:-9}" \
    --center-light-power "${LAMP_P:-1.0}" \
    --sun-power "${SUN_P:-0.0}" \
    --part-colors \
    --collision boxes --collision-res "${RES:-0.05}" \
    --shadowsize "${SHADOW:-4096}" \
    "$@"

if [ "$DRONE_FLAG" = "1" ] || [ "${DRONE:-0}" = "1" ]; then
    "$VENV_PY" build/build_x500.py --merge
fi
