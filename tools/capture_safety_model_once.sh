#!/usr/bin/env bash
set -euo pipefail

ts="$(date +%Y%m%d-%H%M%S)"
outdir="/opt/kilo7/logs/phase_4/${ts}-safety-model-once"
mkdir -p "$outdir"

echo "Capturing one-shot from /kilo/state/safety_model to $outdir/safety_model_once.json" >&2

source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash

ros2 topic echo /kilo/state/safety_model std_msgs/msg/String --once >"$outdir/safety_model_once.json"

echo "Wrote: $outdir/safety_model_once.json" >&2
