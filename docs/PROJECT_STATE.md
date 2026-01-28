CURRENT PHASE:
Phase 1 — Bring-up (Steps 1.4, 1.5 & 1.6 COMPLETE ✅)

CURRENT GOAL:
Step 1.8 — Prepare UI truth + soak (PASS, 2026-01-27, CT-2026-01-27)

LAST CONFIRMED WORKING STATE:
✅ Step 1.4 PASS — Safety Gate ROS Authority (logic-only)
- Published on `/kilo/state/safety_json` (schema: state_safety_v1)
- Accepts unlock/stop commands via ROS topics
- All topics active, no import errors

✅ Step 1.5 PASS — Relay Kill Hardware Enforcement  
- GPIO17 in rpi_gpio mode, controls relay fail-closed
- Published on `/kilo/hw/relay_status_json` (schema: hw_relay_v1)
- Boots to KILL state (safe), GPIO transitions verified
- Config: relay_mode=rpi_gpio, relay_kill_level_low=false (HIGH=KILL)

✅ Step 1.6 PASS — Control Arbitration Authority Chain
- Authority model locked in docs/AUTHORITY_CHAIN.md (immutable)
- Single publisher enforced on each truth topic:
  - `/kilo/state/safety_json` (Safety Gate only)
  - `/kilo/state/control_json` (Control PWM only)
- MQTT→ROS schema validation enforced (Step 1.6 TICKET 2):
  - Wrong schema on cmd topics dropped without refreshing TTL
  - cmd_drive_v1, cmd_stop_v1, cmd_heartbeat_v1, cmd_unlock_v1 required
- Control PWM applies Safety Gate decisions (armed=false when gate denies)
- Integration tests (TICKET 3) verify all 6 invariants: PASS
  - Single publisher checks
  - Gate deny → throttle 0.0 exactly
  - Schema mismatch handling
  - Relay state reflection

REPO / BASELINE:
- `/opt/kilo7` is runtime and build source
- Auto-pull enabled (ff-only, clean-tree only)
- Current baseline: commit 8f567fa + Step 1.6 changes (schema enforcement, armed field)
- PCA9685 verified on I2C bus @0x40
  - ESC/throttle channel = 0
  - Steering channel = 1
  - 50Hz operation confirmed
  - All-channels-off command deterministic
- GPIO17 reserved for ESC kill relay control
- Hardware relay kill-path installed inline with ESC power:
  - Battery+ → COM, ESC+ → NO
  - Active-LOW trigger (LOW=RUN, HIGH=KILL)
  - Default boot state = KILL
- Validated:
  - Manual RUN/KILL toggle works
  - Reboot returns to KILL
  - Software crash cannot defeat kill-path

Step 1.4 — Safety Gate bring-up (PASS, ROS authority, logic-only)
- Authority: ROS Safety Gate (`kilo7-safety-gate.service` / `kilo_core/safety_gate`)
- Publishes authoritative safety truth on: `/kilo/state/safety_json`
- Legacy HTTP Safety Gate is forbidden:
  - `kilo-safety-gate.service` is masked and must remain masked
  - No listeners on TCP 8098
- Verified behavior (logic-only, no motion):
  - No heartbeat → DENY / LOSS_OF_COMMAND (fail-closed)
  - Heartbeat with all OK → ALLOW / OK
  - Stop latch → DENY / EXPLICIT_STOP
  - Component fault + heartbeat → DENY / COMPONENT_MISSING (or equivalent)
  - Priority confirmed:
    - LOSS_OF_COMMAND overrides other faults
    - EXPLICIT_STOP overrides all when set

Step 1.5 — Relay Kill Hardware Enforcement (PASS, hardware authority)
- Hardware relay on GPIO17 controls ESC power
- Fail-closed by default (HIGH=KILL, LOW=RUN)
- RelayKill node publishes state on `/kilo/hw/relay_status_json`
- Control PWM reads relay state and enforces: relay_killed → throttle=0.0
- Software cannot defeat (relay is in series with ESC power)

Step 1.6 — Control Arbitration Authority Chain (PASS, application layer)
- TICKET 1: Authority Chain Documentation
  - docs/AUTHORITY_CHAIN.md defines truth/request topic contracts
  - Single-authority model enforced: only Safety Gate authorizes motion
  - Schema versions locked (cmd_*_v1, state_*_v1)
  
- TICKET 2: MQTT→ROS Schema Enforcement
  - mqtt_bridge validates schema_version on all critical command topics
  - Mismatched schema → dropped, no TTL refresh (command TTL enforces command liveness)
  - ts_ms required and validated (already implemented)
  - Range checks on steer/throttle (clamp, do not drop)
  
- TICKET 3: Integration Tests
  - robot/test_step_1_6_invariants.py verifies 6 step-1.6 invariants
  - All tests PASS ✓
  - Run: `source /opt/ros/humble/setup.bash && source /opt/kilo7/robot/ros_ws/install/setup.bash && python3 /opt/kilo7/robot/test_step_1_6_invariants.py`

ACCEPTANCE CRITERIA (All Met):
A) "No other process can claim safety authority"
   ✓ Single publisher on /kilo/state/safety_json (kilo_safety_gate only)
   ✓ Legacy HTTP gate masked; no :8098 listener
   ✓ Control reads only /kilo/state/safety_json for gate truth
   ✓ Test: `ros2 topic info /kilo/state/safety_json --verbose` shows 1 publisher

B) "UI/consumers cannot misinterpret non-authoritative signals as safe"
   ✓ Two truth topics clearly defined: safety_json (safe_to_move), control_json (armed/applied)
   ✓ Request topics (/kilo/cmd/*) are explicitly non-authoritative
   ✓ Schema versioning enforces contract (wrong schema = ignored)
   ✓ Control publishes gate_safe_to_move and gate_reason caches for audit trail

C) "When Safety Gate denies, throttle is forced neutral (0.0)"
   ✓ control_pwm.py: `if not self._gate_safe_to_move: applied_throttle = 0.0`
   ✓ armed field set to: `bool(applied_throttle != 0.0)` (reflects actual motion)
   ✓ Test verified: gate deny → throttle exactly 0.0, armed false
   ✓ Relay also enforces at hardware level: relay_killed → throttle=0.0

WHAT IS BROKEN (KNOWN):
None — Steps 1.4, 1.5 & 1.6 complete and passing

WHAT IS UNTESTED:
- Step 1.7+ (Perception, Mapping, Navigation, Speed-aware safety)
- MQTT bridge under load (full publish/subscribe integration)
- Phone sensor integration (when added)
- Multi-node failure scenarios (one node crash, system recovery)

Step 1.7 — Voice Intent + Phone IMU (PROVEN end-to-end)
- mqtt_bridge configured to use LAN broker via mDNS: host=kilo-dev.local
- Broker accessible over LAN (0.0.0.0:1883) and advertised via Avahi (_mqtt._tcp)
- Verified STOP intent (cmd_intent_v1) → Safety Gate publishes EXPLICIT_STOP denial
- Verified IMU (phone_imu_v1) → bridged to ROS on /kilo/phone/imu_json
- UI must derive emotion/lockout from authoritative topics only (safety/control)

Additions (2026-01-23):
- Observability: `mqtt_bridge` now emits alerts on schema mismatches for critical inbound topics (intent, IMU, drive, stop, heartbeat, unlock). Drops still occur and TTL is not refreshed.
- Tooling: Added consolidated Step 1.7 verification harness at /opt/kilo7/tools/step_1_7_verify.sh to capture MQTT and ROS truth outputs into logs under /opt/kilo7/logs/step_1_7.
 - Stability: Rate limiting for repetitive MQTT error alerts (type+topic keyed) added to `mqtt_bridge` to reduce log spam. Configurable via `mqtt.alert_min_interval_s` (default 2s).

Hardenings & Verifications (2026-01-23 PM):
- Heartbeat reliability: Bridge subscribes to `kilo/cmd/heartbeat` at QoS 1 and forwards to `/kilo/cmd/heartbeat_json`. Manual probe confirms ROS echo present with `cmd_heartbeat_v1`.
- Clean shutdown: Fixed bridge shutdown to stop/disconnect MQTT loop before destroying ROS node to avoid rclpy invalid-handle errors on service restarts.
- Current truth state under test:
  - Safety: `/kilo/state/safety_json` shows `EXPLICIT_STOP` latched (expected after STOP intent publish).
  - Control: `/kilo/state/control_json` shows `locked=true`, `locked_reason=HEARTBEAT_STALE`, `relay_killed=true` (hardware kill default). Note: `locked_reason=HEARTBEAT_STALE` persists until explicit unlock; fresh heartbeat does not auto-unlock (by design).

Pending verifications for Step 1.7 PASS:
 Verified Step 1.7 artifacts:
 - CLEAR_STOP path live via MQTT→ROS bridge; Safety Gate latch clear per design.
 - Combined path: STOP → CLEAR_STOP → UNLOCK → HEARTBEAT behaves per spec.
 - Relay: rpi_gpio shows RUN (LOW); Control mirrors relay_killed=false.
 - Safety: safe_to_move=true (OK) when latched cleared and override satisfied.
 - Control: unlock after UNLOCK request; publishes `locked` and `locked_reason` transitions.

NEXT CONCRETE STEP:
- Step 1.8 scope proposal (to confirm):
  - UI truth derivation: drive UI lock/emotion strictly from `/kilo/state/safety_json` and `/kilo/state/control_json`.
  - Soak tests: sustained IMU + intent flows; confirm no raw drive paths exposed.
  - Add invariants for intent handling and UI truth mapping.
  - Keep offline-first and single-authority guardrails.

Additions (2026-01-27):
  - Cleanup completed: archived and removed old install `/opt/kilo7_old_20260119_143131` and venvs at `/opt/kilo7/.venv`.
  - Archives stored under `/opt/kilo7/logs/cleanup-20260127-103113` (verified contents).
  - Systemd inventory shows all `kilo7-*` units using `WorkingDirectory=/opt/kilo7`; legacy `kilo-safety-gate.service` remains masked; no stale path references (`/opt/kilo/kilo7`, `/opt/kilo7_old_*`).
  - Tests:
    - `robot/test_step_1_6_invariants.py` → PASS (6/6) with publisher-only checks.
    - `robot/test_step_1_8_ui_truth.py` → PASS (3/3) aligned to documented precedence.
  - Repo hygiene: `.gitignore` updated to ignore Python venvs and caches.

Additions (2026-01-28):
  - Phase 5 mapping contracts defined (state_mapping_v1) and stub publisher added (truth-only).
  - New topic: `/kilo/state/mapping_json` with map status, localization pose validity, and quality fields.
  - Test added: `robot/test_phase5_mapping_summary.py` → PASS.

```
