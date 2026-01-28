#!/usr/bin/env bash
set -euo pipefail

# KILO .7 verification runner: runs phase tests with timeouts and saves logs

timestamp=$(date +"%Y%m%d-%H%M%S")
outdir="/opt/kilo7/logs/phase_6/${timestamp}-verification"
mkdir -p "$outdir"

echo "Logs: $outdir"

set +u
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
set -u

run() {
  name="$1"; shift || true
  base=$(basename "$name" .py)
  log="$outdir/${base}.log"
  echo "[Run] $base" | tee -a "$outdir/summary.txt"
  if /usr/bin/timeout 120s python3 "$name" &>"$log"; then
    echo "PASS  $base" | tee -a "$outdir/summary.txt"
  else
    code=$?
    echo "FAIL  $base (exit=$code)" | tee -a "$outdir/summary.txt"
  fi
}

run /opt/kilo7/robot/test_step_1_6_invariants.py
run /opt/kilo7/robot/test_step_1_8_ui_truth.py
run /opt/kilo7/robot/test_step_2_3_imu_stale.py
run /opt/kilo7/robot/test_step_2_4_rollover.py
run /opt/kilo7/robot/test_step_2_5_impact.py
run /opt/kilo7/robot/test_phase3_perception_summary.py
run /opt/kilo7/robot/test_phase3_perception_validity.py
run /opt/kilo7/robot/test_phase4_safety_model.py
run /opt/kilo7/robot/test_phase4_enforcement_wiring.py
run /opt/kilo7/robot/test_phase5_mapping_summary.py

# Phase 6
if [ -f /opt/kilo7/robot/test_phase6_navigation_summary.py ]; then
  run /opt/kilo7/robot/test_phase6_navigation_summary.py
else
  echo "SKIP  test_phase6_navigation_summary.py (file not found)" | tee -a "$outdir/summary.txt"
fi

echo "Done. Summary at $outdir/summary.txt"
