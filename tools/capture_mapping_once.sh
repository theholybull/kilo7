#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source ${KILO_ROS_WS:-/opt/kilo7/robot/ros_ws}/install/setup.bash

TS=$(date +%Y%m%d-%H%M%S)
OUT_DIR=/opt/kilo7/logs/phase_5/${TS}-mapping-once
mkdir -p "$OUT_DIR"

python3 /opt/kilo7/tools/echo_json_once.py /kilo/state/mapping_json | tee "$OUT_DIR/mapping_once.json"
python3 /opt/kilo7/tools/ui_truth_probe.py --once | tee "$OUT_DIR/ui_truth_once.json"

echo "Artifacts: $OUT_DIR"
