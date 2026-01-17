FINAL KNOWN-GOOD RECORD — KILO .7 Backend (Phase 1–2)

This captures the exact, working state you just validated. Archive this with the clean install pack.

A) systemd — Final ExecStart (all four units)

Pattern (identical across services; node name differs):

ExecStart=/bin/bash -lc '
  source ${KILO_ROS_WS:-/opt/kilo7/robot/ros_ws}/install/setup.bash &&
  if [ -n "${KILO_CONFIG:-}" ]; then
    exec ros2 run kilo_core <NODE> --ros-args -p config:="${KILO_CONFIG}";
  else
    exec ros2 run kilo_core <NODE>;
  fi
'


Nodes:

<NODE>=control_pwm

<NODE>=safety_gate

<NODE>=relay_kill

<NODE>=mqtt_bridge

Why this is locked-good:

Always sources the overlay.

Never emits -p config:= when empty.

Behavior is unchanged when KILO_CONFIG is set.

B) Environment files (present + loaded)
/etc/default/kilo7
# KILO .7 — defaults
KILO_ROS_WS=/opt/kilo7/robot/ros_ws
# Optional; leave unset if not using a config file
# KILO_CONFIG=/opt/kilo7/config/kilo7.yaml

/opt/kilo7/kilo7-backend.env
# KILO .7 — backend env
# Keep minimal; service units source this safely.

# ROS_DOMAIN_ID optional; leave unset unless coordinating domains
# ROS_DOMAIN_ID=0

# Optional config override (only if used)
# KILO_CONFIG=/opt/kilo7/config/kilo7.yaml


Verified by systemd: both files load with ignore_errors=yes.

C) Dependency baseline (must be present)
sudo apt install -y python3-paho-mqtt


Proof: import paho.mqtt.client succeeds; mqtt_bridge stays up.

D) Acceptance evidence (captured)

Services: active / running, NRestarts=0

MQTT truth topics (contract-compliant):

kilo/health → schema_version, ts_ms, ok, build, uptime_s

kilo/state/safety → deny state with OVERRIDE_REQUIRED

kilo/state/control → neutral outputs (throttle=0.0), locked, relay killed

E) CHANGELOG entry (final)
## 2026-01-17 — CT-2026-01-17-RT-001 — Backend runtime stabilization (Phase 1–2)

### Changes
- Added env files: /etc/default/kilo7, /opt/kilo7/kilo7-backend.env
- Installed python3-paho-mqtt
- Hardened systemd ExecStart to omit -p config:= when KILO_CONFIG is empty

### Verification
- All services active, NRestarts=0
- MQTT truth topics observed and contract-compliant

F) Snapshot commands (for future audits)
systemctl is-active kilo7-control kilo7-safety-gate kilo7-relay-kill kilo7-mqtt-bridge
systemctl show kilo7-control kilo7-safety-gate kilo7-relay-kill kilo7-mqtt-bridge \
  -p NRestarts,ActiveState,SubState --no-pager
mosquitto_sub -t kilo/health -C 1 -v
mosquitto_sub -t kilo/state/safety -C 1 -v
mosquitto_sub -t kilo/state/control -C 1 -v


Status: Ticket closed. Phase 1–2 validation complete.
Next work can proceed without touching the contract.
