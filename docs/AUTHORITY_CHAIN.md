# AUTHORITY CHAIN — Step 1.6 Hard Requirements

## Single Authority Model (Non-Negotiable)

KILO.7 enforces **one authority for safety decisions** and **one authority for applied motion state**. No other source may be treated as truth.

---

## ROS Truth Topics (Authoritative — UI May Trust)

### `/kilo/state/safety_json` — Safety Gate Authority

**Owner:** `kilo_safety_gate` process (single publisher, non-negotiable)

**Meaning:** "Is motion allowed right now?" + reason/latch status.

**Required Schema Fields:**
```
schema_version: "state_safety_v1"  (exact match required)
ts_ms: integer                      (timestamp ms)
safe_to_move: bool                  (motion allowed?)
reason: string                      (why: "OK", "EXPLICIT_STOP", "OVERRIDE_REQUIRED", etc.)
latched: bool                       (stop latch active?)
```

**UI Use:** 
- **The only source for "safe_to_move"** — read this field and nothing else for motion permission.
- Do not infer safety from: "heartbeat received", "bridge connected", "cmd age", "unlock sent", etc.

### `/kilo/state/control_json` — Control Authority

**Owner:** `kilo_control_pwm` process (single publisher, non-negotiable)

**Meaning:** "What is actually applied right now?" (locked status, armed state, applied outputs, state caches).

**Required Schema Fields:**
```
schema_version: "state_control_v1"  (exact match required)
ts_ms: integer                       (timestamp ms)
armed: bool                          (throttle != 0.0?)
locked: bool                         (control locked by local logic?)
locked_reason: string                (why: "BOOT_LOCK", "HEARTBEAT_STALE", "RELAY_KILLED", etc.)
applied: {                           (what actually went to PWM)
  steer: float,
  throttle: float                    (MUST be 0.0 exactly when gate denies)
}
gate_safe_to_move: bool              (cache: what Safety Gate said when we published)
gate_reason: string                  (cache: Safety Gate reason)
relay_killed: bool                   (cache: relay hardware state)
relay_reason: string                 (cache: relay reason)
stale_cmd: bool                      (command TTL expired?)
last_cmd_age_ms: integer             (ms since last valid command)
```

**UI Use:**
- **The only source for "armed"** — read this field to know if motion is applied.
- **The only source for "applied outputs"** — read `.applied.steer` and `.applied.throttle` to show actual motion.
- **Do not infer applied state from:** "bridge connected", "cmd received", "unlock sent", "has_cmd", etc.

---

## ROS Request Topics (Never Authoritative — UI Must NOT Treat as Truth)

**All topics under `/kilo/cmd/*` are requests, not truth.**

- Requests can be dropped, clamped, ignored, or overridden.
- Requests do not prove anything about motion safety or applied state.
- UI must **never** infer "safe" or "armed" from request topics.

**Topics:**
- `/kilo/cmd/drive_json` — Drive command (can be stale, dropped, clamped)
- `/kilo/cmd/stop_json` — Stop command (latches, can be overridden by unlock)
- `/kilo/cmd/heartbeat_json` — Liveness proof (can expire)
- `/kilo/cmd/unlock_json` — Override assertion (can be ignored if heartbeat stale)
 - `/kilo/cmd/clear_stop_json` — Explicit operator request to clear `EXPLICIT_STOP` latch in Safety Gate (request, not truth). May still result in `OVERRIDE_REQUIRED` depending on config.

---

## MQTT Transport (Just Transport, Not Authority)

**MQTT is a delivery mechanism only.** MQTT messages are requests until they are published to one of the two ROS truth topics above.

- MQTT bridge connection status: **Not authoritative.** Tells you "can we hear commands?" not "is it safe?"
- MQTT message receipt: **Not authoritative.** Tells you "was this forwarded?" not "was this applied?"
- MQTT retain/QoS: No bearing on truth; only on transport reliability.

---

## Hard Requirements (Non-Negotiable)

### Requirement 1: Single Authority on `/kilo/state/safety_json`

**PASS:** Exactly one publisher (the Safety Gate node).

**FAIL:** Multiple publishers, or publisher is not Safety Gate.

**Verification Command:**
```bash
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
ros2 topic info /kilo/state/safety_json
```

Expected output: `1 publisher` shown, and it is the `kilo_safety_gate` node.

**Additional Check (Legacy HTTP Gate Must Be Masked):**
```bash
ss -ltnp | grep ':8098' || echo "OK: no :8098 listener"
systemctl is-enabled kilo-safety-gate.service 2>&1 | grep -q masked && echo "PASS: legacy masked" || echo "FAIL: legacy not masked"
```

### Requirement 2: Single Authority on `/kilo/state/control_json`

**PASS:** Exactly one publisher (the ControlPWM node).

**FAIL:** Multiple publishers, or publisher is not ControlPWM.

**Verification Command:**
```bash
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
ros2 topic info /kilo/state/control_json
```

Expected output: `1 publisher` shown, and it is the `kilo_control_pwm` node.

### Requirement 3: "Deny Clamps Applied Throttle"

**PASS:** When Safety Gate denies motion, Control publishes `applied.throttle` exactly `0.0` and `armed: false`.

**FAIL:** Control does not clamp throttle to 0.0, or `armed` is not false.

**Verification Commands:**
```bash
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash

# Publish STOP command (forces gate denial)
ros2 topic pub -1 /kilo/cmd/stop_json std_msgs/msg/String \
  "{data: '{\"schema_version\":\"cmd_stop_v1\",\"ts_ms\":$(date +%s%3N)}'}"

# Check Safety Gate response (should show safe_to_move: false)
ros2 topic echo /kilo/state/safety_json std_msgs/msg/String --once

# Check Control response (should show applied.throttle: 0.0 and armed: false)
ros2 topic echo /kilo/state/control_json std_msgs/msg/String --once
```

Expected output in control_json:
```json
{
  "schema_version": "state_control_v1",
  ...
  "armed": false,
  "locked": true,
  "applied": {
    "steer": 0.0,
    "throttle": 0.0
  },
  "gate_safe_to_move": false,
  "gate_reason": "EXPLICIT_STOP",
  ...
}
```

---

## Schema Validation Rules (Enforcement at Boundaries)

### MQTT→ROS Bridge (`mqtt_bridge.py`)

1. **Schema version must match exactly.** Unknown or mismatched schema → drop, do not publish.
2. **`ts_ms` must be present.** Missing → drop, do not publish.
3. **Range enforcement:** Clamp steer/throttle to [-1.0, 1.0]; do not drop (clamping is operational).
4. **No retain on request topics.** Inbound subscriptions must use `retain=false`.

### Control Authority (`control_pwm.py`)

1. **Valid drive request = schema match + ts_ms present + fields within range.**
2. **Only valid drive requests refresh command TTL.** Invalid requests do not keep the robot "alive."
3. **Always consult Safety Gate truth.** Control reads `/kilo/state/safety_json` and uses only `safe_to_move` for throttle decision.
4. **Throttle is forced 0.0 when:**
   - `safe_to_move == false` (gate denies), OR
   - `cmd_stale == true` (no valid command in `cmd_ttl_ms`), OR
   - `heartbeat_stale == true` (no heartbeat in `hb_ttl_ms`), OR
   - `relay_killed == true` (hardware relay asserted).

---

## Files Implementing This Model

| File | Authority |
|------|-----------|
| `kilo_core/safety_gate.py` | Publishes `/kilo/state/safety_json` (single authority) |
| `kilo_core/control_pwm.py` | Publishes `/kilo/state/control_json` (single authority) + enforces Safety Gate truth |
| `kilo_core/mqtt_bridge.py` | Validates and forwards requests (not authority) |
| `kilo_core/relay_kill.py` | Publishes `/kilo/hw/relay_status_json` (supporting; not control authority). In rpi_gpio mode, asserts RUN when Safety Gate allows motion and Control's only lock is `RELAY_KILLED` (bootstrap release); does not make motion decisions. |
| `kilo_core/config/kilo.yaml` | Configuration: TTLs, schema versions, thresholds |

---

## Summary

**The truth surface area for Step 1.6 is exactly two topics:**

1. `/kilo/state/safety_json` (Safety Gate) — "Is motion safe?"
2. `/kilo/state/control_json` (Control) — "What is applied?"

**Everything else is request, diagnostic, or supporting.** UI reads only these two topics for safety/motion decisions. Control enforces the Safety Gate's decision at the PWM level. No other process, topic, or transport can claim authority.

This design makes the motion safety model **impossible to bypass and impossible to misread.**
