#!/usr/bin/env bash
set -euo pipefail

# MQTT load/soak: publish heartbeat + IMU at configurable rates

BROKER_HOST=${BROKER_HOST:-kilo-dev.local}
BROKER_PORT=${BROKER_PORT:-1883}
DURATION_S=${DURATION_S:-60}
HB_HZ=${HB_HZ:-10}
IMU_HZ=${IMU_HZ:-5}

TS=$(date +%Y%m%d-%H%M%S)
OUT_DIR=/opt/kilo7/logs/ops/${TS}-mqtt-soak
mkdir -p "$OUT_DIR"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT_DIR/summary.txt"; }

log "Starting MQTT soak: broker=${BROKER_HOST}:${BROKER_PORT}, duration=${DURATION_S}s, hb=${HB_HZ}Hz, imu=${IMU_HZ}Hz"

hb_interval=$(python3 - <<PY
print(1.0/float(${HB_HZ}))
PY
)
imu_interval=$(python3 - <<PY
print(1.0/float(${IMU_HZ}))
PY
)

HB_COUNT_FILE="$OUT_DIR/hb_count.txt"
IMU_COUNT_FILE="$OUT_DIR/imu_count.txt"
HB_ERR_LOG="$OUT_DIR/hb_errors.log"
IMU_ERR_LOG="$OUT_DIR/imu_errors.log"

hb_loop() {
  local count=0
  local start=$(date +%s)
  while [ $(( $(date +%s) - start )) -lt "$DURATION_S" ]; do
    local ts_ms=$(date +%s%3N)
    mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t 'kilo/cmd/heartbeat' -q 0 \
      -m "{\"schema_version\":\"cmd_heartbeat_v1\",\"ts_ms\":${ts_ms}}" 2>>"$HB_ERR_LOG" || true
    count=$((count+1))
    sleep "$hb_interval"
  done
  echo "$count" > "$HB_COUNT_FILE"
}

imu_loop() {
  local count=0
  local start=$(date +%s)
  while [ $(( $(date +%s) - start )) -lt "$DURATION_S" ]; do
    local ts_ms=$(date +%s%3N)
    mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t 'kilo/phone/imu' -q 0 \
      -m "{\"schema_version\":\"phone_imu_v1\",\"ts_ms\":${ts_ms},\"quaternion\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}" 2>>"$IMU_ERR_LOG" || true
    count=$((count+1))
    sleep "$imu_interval"
  done
  echo "$count" > "$IMU_COUNT_FILE"
}

hb_loop &
HB_PID=$!
imu_loop &
IMU_PID=$!

wait "$HB_PID" "$IMU_PID"
hb_count=$(cat "$HB_COUNT_FILE" 2>/dev/null || echo 0)
imu_count=$(cat "$IMU_COUNT_FILE" 2>/dev/null || echo 0)
log "Completed MQTT soak: hb_count=${hb_count}, imu_count=${imu_count}"
log "Artifacts: $OUT_DIR"
