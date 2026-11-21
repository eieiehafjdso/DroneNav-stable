#!/usr/bin/env bash
# 只看模型（不接 PX4）：镜头跟随 x500
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
VENV_PY="/data/MuJoCo/MuJoCo-drones-gym/.venv/bin/python"
[ -x "$VENV_PY" ] || VENV_PY="python3"
export PYTHONPATH="$ROOT/src:${PYTHONPATH:-}"
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
exec "$VENV_PY" src/view.py "$@"
