#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="/opt/kilo7/robot/ros_ws/run/systemd"
DST_DIR="/etc/systemd/system"

units=(
  "kilo7-relay-kill.service"
  "kilo7-safety-gate.service"
  "kilo7-mqtt-bridge.service"
  "kilo7-control.service"
)

echo "[RT-004] Installing units into ${DST_DIR} ..."
for u in "${units[@]}"; do
  sudo install -m 0644 "${SRC_DIR}/${u}" "${DST_DIR}/${u}"
done

echo "[RT-004] systemd daemon-reload ..."
sudo systemctl daemon-reload

echo "[RT-004] enable units ..."
sudo systemctl enable "${units[@]}" >/dev/null

echo "[RT-004] restart units ..."
sudo systemctl restart "${units[@]}"

echo "[RT-004] status summary:"
systemctl is-active "${units[@]}"
