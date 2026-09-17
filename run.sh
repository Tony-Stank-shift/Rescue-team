#!/bin/bash
# 智能救援机器人 上位机启动脚本（RDK）
cd "$(dirname "$0")"
export RUN_MODE="${RUN_MODE:-real}"
export CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyS1}"
export CAM_INDEX="${CAM_INDEX:-0}"
export SKIP_CAMERA_CHECK="${SKIP_CAMERA_CHECK:-1}"
export PYTHONPATH=src
echo "启动: RUN_MODE=$RUN_MODE CHASSIS_PORT=$CHASSIS_PORT CAM_INDEX=$CAM_INDEX SKIP_CAMERA_CHECK=$SKIP_CAMERA_CHECK"
exec python3 -m rescue_robot.main "$@"
