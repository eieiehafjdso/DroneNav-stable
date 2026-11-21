#!/usr/bin/env bash
# 独立启动 MicroXRCEAgent（不用 ros2 launch 也能直接跑）
#
#   用法:  ./launch/start_agent.sh [端口]      # 默认 8888
#
# 作用：把 PX4 的 uORB 桥接成 ROS2 /fmu/* 话题，必须比 PX4 先起、常驻。
# 前提：PX4 里 param set UXRCE_DDS_SYNCT 0（否则一个 topic 都不建）。
set -e

PORT="${1:-8888}"
echo "MicroXRCEAgent udp4 -p ${PORT}  (Ctrl+C 退出)"
exec MicroXRCEAgent udp4 -p "${PORT}"
