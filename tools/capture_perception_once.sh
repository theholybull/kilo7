#!/usr/bin/env bash
set -euo pipefail

ts="$(date +%Y%m%d-%H%M%S)"
outdir="/opt/kilo7/logs/phase_3/${ts}-perception-once"
mkdir -p "$outdir"

echo "Capturing one-shot from /kilo/state/perception_json to $outdir/perception_once.json" >&2

source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash

ros2 topic echo /kilo/state/perception_json std_msgs/msg/String --once >"$outdir/perception_once.json"

echo "Wrote: $outdir/perception_once.json" >&2
