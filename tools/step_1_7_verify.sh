#!/usr/bin/env bash
set -u
LOG_ROOT=/opt/kilo7/logs
LOG_DIR="$LOG_ROOT/step_1_7"
# MQTT broker config (override via env: MQTT_HOST/MQTT_PORT)
MQTT_HOST=${MQTT_HOST:-"kilo-dev.local"}
MQTT_PORT=${MQTT_PORT:-1883}
# Ensure log directories exist and are writable by current user
if [ ! -d "$LOG_ROOT" ]; then
  if ! mkdir -p "$LOG_ROOT" 2>/dev/null; then
    sudo mkdir -p "$LOG_ROOT"
    sudo chown "$USER":"$USER" "$LOG_ROOT"
  fi
fi
if [ ! -d "$LOG_DIR" ]; then
  if ! mkdir -p "$LOG_DIR" 2>/dev/null; then
    sudo mkdir -p "$LOG_DIR"
    sudo chown -R "$USER":"$USER" "$LOG_DIR"
  fi
fi

{
  echo "==== REPO STATE ===="
  git rev-parse --short HEAD || true
  echo "-- git status --porcelain"
  git status --porcelain || true
  echo "-- git diff --stat"
  git diff --stat || true
} | tee "$LOG_DIR/repo_state.txt"

{
  echo "==== SERVICES ===="
  systemctl is-active kilo7-control kilo7-safety-gate kilo7-relay-kill kilo7-mqtt-bridge || true
  ss -ltnp 2>/dev/null | grep ':8098' || echo "✓ OK: no :8098 listener"
  if systemctl is-enabled kilo-safety-gate.service 2>&1 | grep -q masked; then
    echo "✓ PASS: legacy masked"
  else
    echo "✗ FAIL: legacy not masked"
  fi
} | tee "$LOG_DIR/services.txt"

# ROS environment (required for ros2 CLI)
# Avoid unbound variable errors in ROS setup scripts
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f /opt/kilo7/robot/ros_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/kilo7/robot/ros_ws/install/setup.bash
fi
set -u
command -v ros2 >/dev/null 2>&1 || echo "WARN: ros2 not found in PATH; some checks may be skipped" | tee -a "$LOG_DIR/ros_publishers.txt"

# Detect unlock/latch semantics from active config to set expectations
CFG_FILE="/opt/kilo7/robot/ros_ws/src/kilo_core/config/kilo.yaml"
ALLOW_CLEAR_WHILE_OVERRIDE="false"
if [ -f "$CFG_FILE" ]; then
  # extract boolean value; default false
  ALLOW_CLEAR_WHILE_OVERRIDE=$(awk '/^safety:/{flag=1;next}/^[^[:space:]]/{flag=0}flag && /allow_clear_while_override/ {print $2}' "$CFG_FILE" 2>/dev/null | tr -d '\r')
  if [ -z "$ALLOW_CLEAR_WHILE_OVERRIDE" ]; then ALLOW_CLEAR_WHILE_OVERRIDE="false"; fi
fi
echo "unlock_clears_stop (allow_clear_while_override): $ALLOW_CLEAR_WHILE_OVERRIDE" | tee "$LOG_DIR/config_unlock_semantics.txt"

{
  echo "==== ROS TOPIC INFO: /kilo/state/safety_json ==="
  if ! ros2 topic info /kilo/state/safety_json --verbose | sed -n '1,50p'; then
    echo "WARN: /kilo/state/safety_json info unavailable"
  fi
  echo "==== ROS TOPIC INFO: /kilo/state/control_json ==="
  if ! ros2 topic info /kilo/state/control_json --verbose | sed -n '1,50p'; then
    echo "WARN: /kilo/state/control_json info unavailable"
  fi
} | tee "$LOG_DIR/ros_publishers.txt"

# IMU path proof
{
  echo "==== MQTT IMU SAMPLE (10s) ==="
  if command -v mosquitto_sub >/dev/null 2>&1; then
    timeout 10 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -v -t 'kilo/phone/imu' || true
  else
    echo "WARN: mosquitto_sub not found"
  fi
} | tee "$LOG_DIR/mqtt_imu_sample.txt"

{
  echo "==== ROS /kilo/phone/imu_json existence ==="
  if ! ros2 topic list | grep -E '^/kilo/phone/imu_json$'; then echo "✗ FAIL: /kilo/phone/imu_json missing"; fi
  echo "==== ROS /kilo/phone/imu_json info ==="
  ros2 topic info /kilo/phone/imu_json --verbose | sed -n '1,50p' || echo "WARN: imu_json info unavailable"
  echo "==== ROS /kilo/phone/imu_json echo (once) ==="
  timeout 5 ros2 topic echo /kilo/phone/imu_json std_msgs/msg/String --once &
  ECHO_PID=$!
  sleep 0.5
  echo "==== Publish valid phone_imu_v1 (quaternion) ==="
  TS=$(date +%s%3N)
  if command -v mosquitto_pub >/dev/null 2>&1; then
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 0 -t 'kilo/phone/imu' \
      -m "{\"schema_version\":\"phone_imu_v1\",\"ts_ms\":$TS,\"quaternion\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}"
  else
    echo "WARN: mosquitto_pub not found"
  fi
  wait $ECHO_PID 2>/dev/null || echo "WARN: no imu_json message echoed"
} | tee "$LOG_DIR/ros_imu_bridge.txt"

# Heartbeat chain proof
{
  echo "==== MQTT heartbeat monitor (5s) ==="
  if command -v mosquitto_sub >/dev/null 2>&1; then
    timeout 5 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -v -t 'kilo/cmd/heartbeat' || true
  else
    echo "WARN: mosquitto_sub not found"
  fi
  echo "==== Publish test heartbeats (2Hz for 3s, QoS1) ==="
  echo "==== ROS /kilo/cmd/heartbeat_json echo (once) ==="
  timeout 5 ros2 topic echo /kilo/cmd/heartbeat_json std_msgs/msg/String --once &
  HB_ECHO_PID=$!
  sleep 0.5
  if command -v mosquitto_pub >/dev/null 2>&1; then
    for i in $(seq 1 6); do
      TS=$(date +%s%3N)
      mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t 'kilo/cmd/heartbeat' \
        -m "{\"schema_version\":\"cmd_heartbeat_v1\",\"ts_ms\":$TS}"
      sleep 0.5
    done
  else
    echo "WARN: mosquitto_pub not found"
  fi
  wait $HB_ECHO_PID 2>/dev/null || echo "WARN: no heartbeat_json message echoed"
  echo "==== Control state after heartbeat (once) ==="
  timeout 5 ros2 topic echo /kilo/state/control_json std_msgs/msg/String --once || echo "WARN: no control_json message echoed"
} | tee "$LOG_DIR/heartbeat_chain.txt"

# STOP intent chain
{
  echo "==== MQTT monitor (intent/alerts/state) 8s ==="
  if command -v mosquitto_sub >/dev/null 2>&1; then
    timeout 8 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -v -t 'kilo/cmd/intent' -t 'kilo/alerts' -t 'kilo/state/safety' -t 'kilo/state/control' &
  else
    echo "WARN: mosquitto_sub not found" &
  fi
  SUB_PID=$!
  TS=$(date +%s%3N)
  echo "Publishing STOP intent at ts_ms=$TS"
  if command -v mosquitto_pub >/dev/null 2>&1; then
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t kilo/cmd/intent -q 1 -m "{\"schema_version\":\"cmd_intent_v1\",\"ts_ms\":$TS,\"intent\":\"STOP\"}"
  else
    echo "WARN: mosquitto_pub not found"
  fi
  wait $SUB_PID 2>/dev/null || true
  echo "==== ROS safety_json (once) ==="
  timeout 3 ros2 topic echo /kilo/state/safety_json std_msgs/msg/String --once || echo "WARN: no safety_json message echoed"
  echo "==== ROS control_json (python probe) ==="
  python3 - << 'PY'
import rclpy, json
from std_msgs.msg import String
rclpy.init()
node=rclpy.create_node('chk')
msg=None
node.create_subscription(String, '/kilo/state/control_json', lambda m: globals().__setitem__('msg', m), 10)
rclpy.spin_once(node, timeout_sec=2)
print('msg_present:', bool(globals().get('msg')))
if globals().get('msg'):
    d=json.loads(globals()['msg'].data)
    print('applied.throttle:', d.get('applied',{}).get('throttle'))
    print('gate_reason:', d.get('gate_reason'))
    print('locked:', d.get('locked'), 'locked_reason:', d.get('locked_reason'))
PY
} | tee "$LOG_DIR/stop_chain.txt"

# Schema violation alert
{
  echo "==== Schema violation test ==="
  if command -v mosquitto_sub >/dev/null 2>&1; then
    timeout 6 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -v -t 'kilo/alerts' &
  else
    echo "WARN: mosquitto_sub not found" &
  fi
  SUB_PID=$!
  TS=$(date +%s%3N)
  echo "Publishing WRONG_v1 intent at ts_ms=$TS"
  if command -v mosquitto_pub >/dev/null 2>&1; then
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t kilo/cmd/intent -q 1 -m "{\"schema_version\":\"WRONG_v1\",\"ts_ms\":$TS,\"intent\":\"STOP\"}"
  else
    echo "WARN: mosquitto_pub not found"
  fi
  wait $SUB_PID 2>/dev/null || true
} | tee "$LOG_DIR/schema_alert.txt"

# Fail-fast drive publication
{
  echo "==== Drive publication check ==="
  if command -v mosquitto_sub >/dev/null 2>&1; then
    timeout 12 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -v -t 'kilo/cmd/drive' && echo "✗ FAIL: app is publishing raw drive" || echo "✓ PASS: no drive messages observed"
  else
    echo "WARN: mosquitto_sub not found"
  fi
} | tee "$LOG_DIR/drive_check.txt"

# Unlock latch behavior check (expectations depend on config)
{
  echo "==== UNLOCK latch behavior check ==="
  echo "Config allow_clear_while_override=$ALLOW_CLEAR_WHILE_OVERRIDE"
  # Force STOP first to ensure latched baseline
  TS=$(date +%s%3N)
  if command -v mosquitto_pub >/dev/null 2>&1; then
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t kilo/cmd/stop -q 1 -m "{\"schema_version\":\"cmd_stop_v1\",\"ts_ms\":$TS}"
  fi
  sleep 0.2
  echo "Safety (pre-unlock)"
  timeout 3 ros2 topic echo /kilo/state/safety_json std_msgs/msg/String --once || true
  # Publish UNLOCK via ROS directly (bypasses MQTT variability)
  ros2 topic pub -1 /kilo/cmd/unlock_json std_msgs/msg/String "{data: '{\"schema_version\":\"cmd_unlock_v1\",\"ts_ms\":$(date +%s%3N)}'}" >/dev/null 2>&1 || true
  sleep 0.2
  echo "Safety (post-unlock)"
  OUT=$(timeout 3 ros2 topic echo /kilo/state/safety_json --once 2>/dev/null || true)
  echo "$OUT"
  if echo "$ALLOW_CLEAR_WHILE_OVERRIDE" | grep -qi '^true$'; then
    echo "$OUT" | grep -q '"latched":false' && echo "✓ PASS: UNLOCK cleared STOP (per config)" || echo "✗ FAIL: UNLOCK did not clear STOP though config allows it"
  else
    echo "$OUT" | grep -q '"latched":true' && echo "✓ PASS: UNLOCK did not clear STOP (per config)" || echo "✗ FAIL: UNLOCK cleared STOP though config forbids it"
  fi
} | tee "$LOG_DIR/unlock_semantics.txt"

# Clear-stop behavior check (explicit operator reset)
{
  echo "==== CLEAR_STOP behavior check ==="
  # Force STOP to set latched
  TS=$(date +%s%3N)
  if command -v ros2 >/dev/null 2>&1; then
    ros2 topic pub -1 /kilo/cmd/stop_json std_msgs/msg/String "{data: '{\"schema_version\":\"cmd_stop_v1\",\"ts_ms\":$(date +%s%3N)}'}" >/dev/null 2>&1 || true
  fi
  sleep 0.2
  echo "Safety (pre-clear-stop)"
  timeout 3 ros2 topic echo /kilo/state/safety_json --once || true
  # Publish CLEAR_STOP via ROS
  ros2 topic pub -1 /kilo/cmd/clear_stop_json std_msgs/msg/String "{data: '{\"schema_version\":\"cmd_clear_stop_v1\",\"ts_ms\":$(date +%s%3N)}'}" >/dev/null 2>&1 || true
  sleep 0.2
  echo "Safety (post-clear-stop)"
  OUT=$(timeout 3 ros2 topic echo /kilo/state/safety_json --once 2>/dev/null || true)
  echo "$OUT"
  echo "$OUT" | grep -q '"latched":false' && echo "✓ PASS: CLEAR_STOP cleared STOP latch" || echo "✗ FAIL: CLEAR_STOP did not clear STOP latch"
} | tee "$LOG_DIR/clear_stop_semantics.txt"

echo "Logs written to $LOG_DIR"