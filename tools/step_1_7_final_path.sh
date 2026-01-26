#!/usr/bin/env bash
set -euo pipefail

# Source ROS environment (disable nounset during setup scripts)
set +u
source /opt/ros/humble/setup.bash || true
source /opt/kilo7/robot/ros_ws/install/setup.bash || true
set -u

LOG_DIR="/opt/kilo7/logs/step_1_7"
LOG_FILE="$LOG_DIR/final_path.txt"
mkdir -p "$LOG_DIR"

now_ts_ms() {
  date +%s%3N
}

ts() {
  date -Iseconds
}

write() {
  echo "[$(ts)] $1" | tee -a "$LOG_FILE"
}

snapshot() {
  write "Safety snapshot:" && ros2 topic echo /kilo/state/safety_json --once | tee -a "$LOG_FILE"
  write "Control snapshot:" && ros2 topic echo /kilo/state/control_json --once | tee -a "$LOG_FILE"
}

publish_json() {
  local topic="$1"
  local json="$2"
  write "Publish -> ${topic}: ${json}"
  # std_msgs/msg/String with YAML mapping form
  ros2 topic pub --once "$topic" std_msgs/msg/String "data: '${json}'" >/dev/null
}

write "Starting combined path check: STOP → CLEAR_STOP → UNLOCK → HEARTBEAT"

# STOP
stop_json="{\"schema_version\":\"cmd_stop_v1\",\"ts_ms\":$(now_ts_ms)}"
publish_json "/kilo/cmd/stop_json" "$stop_json"
sleep 0.3
snapshot

# CLEAR_STOP
clear_json="{\"schema_version\":\"cmd_clear_stop_v1\",\"ts_ms\":$(now_ts_ms)}"
publish_json "/kilo/cmd/clear_stop_json" "$clear_json"
sleep 0.3
snapshot

# UNLOCK
unlock_json="{\"schema_version\":\"cmd_unlock_v1\",\"ts_ms\":$(now_ts_ms)}"
publish_json "/kilo/cmd/unlock_json" "$unlock_json"
sleep 0.3
snapshot

# HEARTBEAT
heartbeat_json="{\"schema_version\":\"cmd_heartbeat_v1\",\"ts_ms\":$(now_ts_ms)}"
publish_json "/kilo/cmd/heartbeat_json" "$heartbeat_json"
sleep 0.3
snapshot

write "Combined path check complete. Logs captured at ${LOG_FILE}"