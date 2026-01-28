#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source ${KILO_ROS_WS:-/opt/kilo7/robot/ros_ws}/install/setup.bash

TS=$(date +%Y%m%d-%H%M%S)
LOG_DIR=/opt/kilo7/logs/phase_5
OUT_DIR="$LOG_DIR/${TS}-mapping-soak"
mkdir -p "$OUT_DIR"

MAP_OUT="$OUT_DIR/mapping_truth.jsonl"
UI_OUT="$OUT_DIR/ui_truth.jsonl"

echo "Starting mapping truth soak (60s) -> $OUT_DIR"
ros2 run kilo_core mapping_summary &
MAP_PID=$!
trap 'kill $MAP_PID 2>/dev/null || true' EXIT
START=$(date +%s)
while [[ $(($(date +%s) - START)) -lt 60 ]]; do
  python3 /opt/kilo7/tools/echo_json_once.py /kilo/state/mapping_json >> "$MAP_OUT" || true
  python3 /opt/kilo7/tools/ui_truth_probe.py --once >> "$UI_OUT" || true
  sleep 1
done

echo "Soak complete. Artifacts: $OUT_DIR"
