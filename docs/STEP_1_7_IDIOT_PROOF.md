# Step 1.7 — Voice Intent + Phone IMU: Exact Examples

This is a copy‑paste guide with exact file paths, JSON payloads, and command examples. Follow exactly as written.

## Broker and Topics
- MQTT Broker: `kilo-dev.local:1883`
- Request topics (published by phone/app):
  - `kilo/cmd/heartbeat` — schema `cmd_heartbeat_v1`
  - `kilo/cmd/intent` — schema `cmd_intent_v1`
  - `kilo/phone/imu` — schema `phone_imu_v1`
- ROS fanout (published by bridge):
  - `/kilo/cmd/heartbeat_json`
  - `/kilo/cmd/intent_json`
  - `/kilo/phone/imu_json`
- Truth topics (ROS only):
  - `/kilo/state/safety_json` (Safety Gate)
  - `/kilo/state/control_json` (Control PWM)
- Health:
  - `kilo/health` — retained summary including `rx_counts`

## Robot Config (Bridge)
- File: [robot/ros_ws/src/kilo_core/config/kilo.yaml](robot/ros_ws/src/kilo_core/config/kilo.yaml)
- Required keys:

```yaml
mqtt:
  host: "kilo-dev.local"
  port: 1883
  keepalive_s: 20
  ignore_drive_from_mqtt: true
```

After changes:

```bash
sudo systemctl restart kilo7-mqtt-bridge
```

## Exact JSON Payloads (copy/paste)

Heartbeat (2–4 Hz):

```json
{"schema_version":"cmd_heartbeat_v1","ts_ms":1769196000000}
```

Intent — STOP:

```json
{"schema_version":"cmd_intent_v1","ts_ms":1769196000000,"intent":"STOP"}
```

Intent — STATUS:

```json
{"schema_version":"cmd_intent_v1","ts_ms":1769196000000,"intent":"STATUS"}
```

Intent — SET_MODE (args example):

```json
{"schema_version":"cmd_intent_v1","ts_ms":1769196000000,"intent":"SET_MODE","args":{"mode":"ROAM"}}
```

Phone IMU (Quaternion, ~4 Hz):

```json
{"schema_version":"phone_imu_v1","ts_ms":1769196000000,
 "quaternion":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}
```

Notes:
- `schema_version` must match exactly.
- `ts_ms` is milliseconds since epoch. The robot computes staleness by its own clock.
- IMU may use roll/pitch/yaw instead of quaternion; at least one orientation representation is required.

## MQTT Publish Commands (CLI)

Heartbeat (single shot):

```bash
ts_ms=$(date +%s%3N)
mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/cmd/heartbeat' -q 1 -m "{\"schema_version\":\"cmd_heartbeat_v1\",\"ts_ms\":$ts_ms}"
```

Heartbeat (2 Hz loop, Ctrl+C to stop):

```bash
while true; do ts_ms=$(date +%s%3N); mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/cmd/heartbeat' -q 1 \
  -m "{\"schema_version\":\"cmd_heartbeat_v1\",\"ts_ms\":$ts_ms}"; sleep 0.5; done
```

Intent — STOP:

```bash
ts_ms=$(date +%s%3N)
mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/cmd/intent' -q 1 -m "{\"schema_version\":\"cmd_intent_v1\",\"ts_ms\":$ts_ms,\"intent\":\"STOP\"}"
```

Intent — SET_MODE ROAM:

```bash
ts_ms=$(date +%s%3N)
mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/cmd/intent' -q 1 -m "{\"schema_version\":\"cmd_intent_v1\",\"ts_ms\":$ts_ms,\"intent\":\"SET_MODE\",\"args\":{\"mode\":\"ROAM\"}}"
```

Phone IMU (single shot):

```bash
ts_ms=$(date +%s%3N)
mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/phone/imu' -q 0 -m "{\"schema_version\":\"phone_imu_v1\",\"ts_ms\":$ts_ms,\"quaternion\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}"
```

## Verification Commands

ROS sourcing (required before any `ros2` command):

```bash
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
```

Check topics exist:

```bash
ros2 topic list
```

Check IMU publisher (bridge):

```bash
ros2 topic info /kilo/phone/imu_json --verbose
```

Echo IMU for 10 seconds:

```bash
timeout 10 ros2 topic echo /kilo/phone/imu_json
```

Health (retained) with RX counters:

```bash
mosquitto_sub -h kilo-dev.local -p 1883 -t 'kilo/health' -v -C 1 -q 1 -W 5
```

## Safety Expectations
- Missing heartbeat: Safety Gate sets `safe_to_move=false`; `control_pwm` applies `throttle=0.0` until heartbeat resumes.
- STOP intent: Immediate denial; `safe_to_move=false`; applied throttle `0.0`.
- Drive from MQTT: Blocked by bridge when `ignore_drive_from_mqtt: true`.

## Optional: Trigger a Schema Violation (for diagnostics)
- Invalid intent version (should be dropped; may alert if enabled):

```bash
ts_ms=$(date +%s%3N)
mosquitto_pub -h kilo-dev.local -p 1883 -t 'kilo/cmd/intent' -q 1 -m "{\"schema_version\":\"cmd_intent_v99\",\"ts_ms\":$ts_ms,\"intent\":\"STATUS\"}"
```

Watch alerts (if enabled):

```bash
mosquitto_sub -h kilo-dev.local -p 1883 -t 'kilo/alerts' -v
```

## Service Management
- Restart bridge: `sudo systemctl restart kilo7-mqtt-bridge`
- Check status: `systemctl status kilo7-mqtt-bridge`

## Where Things Live
- Bridge Python: [robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py](robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py)
- Shared utils: [robot/ros_ws/src/kilo_core/kilo_core/util.py](robot/ros_ws/src/kilo_core/kilo_core/util.py)
- Node configs: [robot/ros_ws/src/kilo_core/config](robot/ros_ws/src/kilo_core/config)
- Verification script: [tools/step_1_7_verify.sh](tools/step_1_7_verify.sh)
- Authority docs: [docs/AUTHORITY_CHAIN.md](docs/AUTHORITY_CHAIN.md)
- Interface contract: [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md)
