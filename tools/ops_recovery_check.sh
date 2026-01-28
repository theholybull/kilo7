#!/usr/bin/env bash
set -euo pipefail

TS=$(date +%Y%m%d-%H%M%S)
OUT_DIR=/opt/kilo7/logs/ops/${TS}-recovery
mkdir -p "$OUT_DIR"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT_DIR/summary.txt"; }

log "Restarting core services..."
sudo systemctl restart kilo7-safety-gate kilo7-control kilo7-relay-kill kilo7-mqtt-bridge || true
sleep 3

log "Service status:"
for svc in kilo7-safety-gate kilo7-control kilo7-relay-kill kilo7-mqtt-bridge; do
  systemctl --no-pager status "$svc" | sed -n '1,12p' | tee -a "$OUT_DIR/${svc}.status.txt" || true
  systemctl is-active "$svc" | tee -a "$OUT_DIR/${svc}.active.txt" || true
  systemctl is-enabled "$svc" | tee -a "$OUT_DIR/${svc}.enabled.txt" || true
done

log "Topic publisher checks:"
set +u
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
set -u
for topic in /kilo/state/safety_json /kilo/state/control_json /kilo/hw/relay_status_json; do
  ros2 topic info "$topic" --verbose | tee -a "$OUT_DIR/topic_${topic//\//_}.txt" || true
done

log "Recovery check complete. Artifacts: $OUT_DIR"
