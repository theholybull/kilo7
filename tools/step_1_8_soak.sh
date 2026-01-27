#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source ${KILO_ROS_WS:-/opt/kilo7/robot/ros_ws}/install/setup.bash

LOG_DIR=/opt/kilo7/logs/step_1_7
OUT="$LOG_DIR/ui_truth_soak.jsonl"
mkdir -p "$LOG_DIR"
echo "Starting UI truth soak (60s) -> $OUT"
START=$(date +%s)
while [[ $(($(date +%s) - START)) -lt 60 ]]; do
  python3 /opt/kilo7/tools/ui_truth_probe.py --once >> "$OUT" || true
  sleep 1
done
echo "Soak complete"
