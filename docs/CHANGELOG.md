# CHANGE_LOG � KILO .7 (Backend)

Rules:
- Log by change ticket (CT-YYYY-MM-DD-RT-###).
- Must include scope, impact, files changed, and verification.
- Contract: INTERFACE_CONTRACTS � KILO .7 v1.2 (additive-only unless explicitly approved).

---

## CT-2026-02-03-RT-033 — Fresh build rebuild script + new machine steps

Date: 2026-02-03
Scope: Rebuild tooling + docs

Change:
- Added fresh rebuild script for Ubuntu 22.04 bring-up.
- Expanded NEW_MACHINE.md with concrete rebuild steps.
- Logged current rebuild readiness in PROJECT_STATE.

Impact:
- Repeatable rebuild process for future fresh installs.

Files changed (repo):
- tools/fresh_build_rebuild.sh
- docs/NEW_MACHINE.md
- docs/PROJECT_STATE.md
- docs/CHANGELOG.md

Verification:
- Script created and made executable; manual run confirmed on current build.

## CT-2026-02-03-RT-034 — Restore parts list (post-crash)

Date: 2026-02-03
Scope: Docs only

Change:
- Restored parts list doc with confirmed chassis/cameras and placeholders for pending links.

Impact:
- Single source for BOM restored after crash.

Files changed (repo):
- docs/PARTS_LIST.md
- docs/PROJECT_STATE.md
- docs/CHANGELOG.md

Verification:
- Docs only.

## CT-2026-02-03-RT-035 — Add pending parts links

Date: 2026-02-03
Scope: Docs only

Change:
- Added two pending parts links (ASIN B08T259X2H, B08CSJXYWY) to parts list.

Impact:
- Parts list updated with user-provided links for later labeling.

Files changed (repo):
- docs/PARTS_LIST.md
- docs/CHANGELOG.md

Verification:
- Docs only.

## CT-2026-02-03-RT-036 — Add additional parts links

Date: 2026-02-03
Scope: Docs only

Change:
- Added two additional pending parts links (ASIN B07WS5XY63, B01ICN5OAM).

Impact:
- Parts list updated with user-provided links for later labeling.

Files changed (repo):
- docs/PARTS_LIST.md
- docs/CHANGELOG.md

Verification:
- Docs only.

## CT-2026-01-28-RT-032 — Ops hardening: rosbridge stability + MQTT load + recovery checks

Date: 2026-01-28
Scope: Ops tooling + docs

Change:
- Added rosbridge logging dir env and doc note.
- Added MQTT load/soak script.
- Added core service recovery check script.
- Logged ops artifacts in BUILD_LOG and PROJECT_STATE.

Impact:
- Visualizer connection stabilized; ops readiness for load and recovery documented.

Files changed (repo):
- robot/ros_ws/run/systemd/kilo7-rosbridge.service
- tools/mqtt_load_soak.sh
- tools/ops_recovery_check.sh
- docs/VISUALIZER.md
- docs/BUILD_LOG.md
- docs/PROJECT_STATE.md

Verification:
- MQTT soak: `logs/ops/20260128-154927-mqtt-soak/`
- Recovery check: `logs/ops/20260128-155146-recovery/`

## CT-2026-01-28-RT-031 — Lockdown status docs update

Date: 2026-01-28
Scope: Docs only

Change:
- Updated PROJECT_STATE current phase/goal and remaining ops gaps.
- Added lockdown status entry to BUILD_LOG.

Impact:
- Clarifies post‑Phase 7 priorities and remaining tests.

Files changed (repo):
- docs/PROJECT_STATE.md
- docs/BUILD_LOG.md

Verification:
- Docs only.

## CT-2026-01-28-RT-030 — Verification suite fixes + IMU stale test robustness

Date: 2026-01-28
Scope: Verification tooling + Phase 2.3 test robustness

Change:
- Fixed verification runner to tolerate ROS setup unbound variables.
- Hardened IMU staleness test to clear explicit stop before asserting staleness.

Impact:
- Full verification suite runs cleanly; IMU stale test no longer flakes on EXPLICIT_STOP latch.

Files changed (repo):
- tools/run_verification_suite.sh
- robot/test_step_2_3_imu_stale.py
- docs/BUILD_LOG.md
- docs/PROJECT_STATE.md

Verification:
- Full suite PASS: `logs/phase_6/20260128-152421-verification/`

## CT-2026-01-28-RT-029 — Phase 7: Docking truth contracts + stub publisher

Date: 2026-01-28
Scope: Docking truth contracts; stub publisher + test (no heavy pipeline)

Change:
- Added `state_docking_v1` contract for `/kilo/state/docking_json`.
- Added stub publisher `kilo_core.docking_summary` (truth-only).
- Added test `robot/test_phase7_docking_summary.py`.
- Added systemd unit template `kilo7-docking-summary.service` (disabled by default).

Impact:
- Provides docking truth fields for UI/observability without enabling docking pipelines.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/docking_summary.py
- robot/ros_ws/src/kilo_core/setup.py
- robot/ros_ws/src/kilo_core/systemd/kilo7-docking-summary.service
- robot/test_phase7_docking_summary.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_phase7_docking_summary.py` → PASS

## CT-2026-01-28-RT-028 — Verification runner script

Date: 2026-01-28
Scope: Test execution tooling

Change:
- Added `tools/run_verification_suite.sh` to execute phase tests sequentially with timeouts and write logs to `logs/phase_6/<timestamp>-verification/`.

Impact:
- Simplifies reproducible local verification and log collection across phases.

Files changed (repo):
- tools/run_verification_suite.sh
- docs/PROJECT_STATE.md

Verification:
- Script created; manual run produces summary and per-test logs under `logs/phase_6/`.

## CT-2026-01-28-RT-027 — Phase 6: Navigation truth contracts + stub publisher

Date: 2026-01-28
Scope: Navigation truth contracts; stub publisher + test (no heavy pipeline)

Change:
- Added `state_navigation_v1` contract for `/kilo/state/navigation_json`.
- Added stub publisher `kilo_core.navigation_summary` (truth-only).
- Added test `robot/test_phase6_navigation_summary.py`.
- Added systemd unit template `kilo7-navigation-summary.service` (disabled by default).

Impact:
- Provides navigation truth fields for UI/observability without enabling navigation pipelines.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/navigation_summary.py
- robot/ros_ws/src/kilo_core/setup.py
- robot/ros_ws/src/kilo_core/systemd/kilo7-navigation-summary.service
- robot/test_phase6_navigation_summary.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_phase6_navigation_summary.py` → PASS

## CT-2026-01-28-RT-026 — Phase 5: Read-only UI Truth Monitor

Date: 2026-01-28
Scope: Phone UI (read-only) for truth-derived lock/emotion

Change:
- Added a static UI under `phone/ui` that connects to rosbridge WebSocket and derives UI lock/emotion from `/kilo/state/safety_json` and `/kilo/state/control_json`.
- UI is read-only and does not send commands.

Impact:
- Provides a lightweight UI for operators without violating single-authority or offline-first guardrails.

Files changed (repo):
- phone/ui/index.html
- phone/ui/app.js
- phone/ui/styles.css
- phone/ui/README.md
- phone/README.md
- docs/PROJECT_STATE.md

Verification:
- Not run (static UI only).

## CT-2026-01-28-RT-025 — Connection info snapshot (rosbridge)

Date: 2026-01-28
Scope: Connection diagnostics / observability logs

Change:
- Captured rosbridge connection info and service status for visualizer setup.

Impact:
- Provides future reference for endpoints and current service health.

Files changed (repo):
- docs/BUILD_LOG.md

Verification:
- Logs: `logs/phase_5/20260128-132803-connection-info/`

## CT-2026-01-28-RT-024 — Phase 5: Mapping UI capture script

Date: 2026-01-28
Scope: Mapping truth one-shot capture for UI/observability

Change:
- Added `tools/capture_mapping_once.sh` to record mapping truth + UI truth snapshot.

Impact:
- Provides a repeatable snapshot for mapping/UI observability without enabling pipelines.

Files changed (repo):
- tools/capture_mapping_once.sh
- docs/BUILD_LOG.md
- docs/PROJECT_STATE.md

Verification:
- Snapshot captured: `logs/phase_5/20260128-132130-mapping-once/`

## CT-2026-01-28-RT-023 — Phase 5: Mapping soak script + UI truth probe update

Date: 2026-01-28
Scope: Mapping truth soak scripts; UI truth probe observability update

Change:
- Added `tools/phase_5_mapping_soak.sh` to capture mapping truth + UI truth over 60s.
- Updated `tools/ui_truth_probe.py` to include mapping observability fields (status/stale/pose_valid/quality).

Impact:
- Enables repeatable mapping truth soak captures without enabling heavy pipelines.

Files changed (repo):
- tools/phase_5_mapping_soak.sh
- tools/ui_truth_probe.py
- docs/BUILD_LOG.md
- docs/PROJECT_STATE.md

Verification:
- Soak completed; logs under `logs/phase_5/20260128-125534-mapping-soak/`

## CT-2026-01-28-RT-022 — Phase 5: Mapping truth contracts + stub publisher

Date: 2026-01-28
Scope: Mapping/localization truth contracts; stub publisher + test (no heavy pipelines)

Change:
- Added `state_mapping_v1` contract for `/kilo/state/mapping_json`.
- Added stub publisher `kilo_core.mapping_summary` (truth-only).
- Added test `robot/test_phase5_mapping_summary.py`.
- Added systemd unit template `kilo7-mapping-summary.service` (disabled by default).

Impact:
- Provides mapping/localization truth fields for UI/observability without enabling mapping pipelines.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/mapping_summary.py
- robot/ros_ws/src/kilo_core/setup.py
- robot/ros_ws/src/kilo_core/systemd/kilo7-mapping-summary.service
- robot/test_phase5_mapping_summary.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_phase5_mapping_summary.py` → PASS

## CT-2026-01-28-RT-021 — Phase 4: Perception enforcement wiring (config-gated)

Date: 2026-01-28
Scope: Safety Gate enforcement wiring to perception truth (default disabled); additive-only fields

Change:
- Added config-gated perception enforcement in Safety Gate (disabled by default).
- Safety Gate subscribes to `/kilo/state/perception_json` and `/kilo/state/safety_model` to deny motion on:
  - hazard level at or above `min_hazard_level`
  - insufficient stop buffer (`stop_distance_m` < `min_stop_distance_m`)
  - stale perception when `stale_denies=true`
- Added additive observability fields in safety truth:
  - `perception_ok`, `perception_age_ms`, `perception_reason`, `stop_distance_m`
- Added Phase 4 enforcement wiring test.

Impact:
- Enforcement is behind `safety.perception_enforcement.enabled=false` by default; no behavioral change unless explicitly enabled.
- Maintains single-authority model; Safety Gate remains the sole motion authority.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/safety_gate.py
- robot/ros_ws/src/kilo_core/config/kilo.yaml
- robot/test_phase4_enforcement_wiring.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_step_1_6_invariants.py` → PASS
- `python3 robot/test_step_1_8_ui_truth.py` → PASS
- `python3 robot/test_phase4_enforcement_wiring.py` → PASS (disabled + enabled scenarios)

## CT-2026-01-27-RT-018 — Infra: rosbridge service template + visualizer doc

Date: 2026-01-27
Scope: Operational docs + systemd unit template for rosbridge (no backend code changes)

Change:
- Added systemd unit template: `robot/ros_ws/run/systemd/kilo7-rosbridge.service` for rosbridge_websocket.
- Added visualizer setup doc: `docs/VISUALIZER.md` with install/enable/verify steps and connection details.

Impact:
- Streamlines enabling WebSocket access to ROS truth topics for offline visualizers.
- Improves reproducibility across new machines.

Verification:
- Service enabled via systemd; port 9090 listener observed.
- Launch attempts report "address already in use" when service active (expected).

## CT-2026-01-28-RT-020 — Phase 3: Perception summary stub (truth-only)

Date: 2026-01-28
Scope: Add perception summary publisher + test + systemd unit template; contracts aligned

Change:
- New node `kilo_core.perception_summary` publishes `/kilo/state/perception_json` (schema `state_perception_v1`).
- Test added: `robot/test_phase3_perception_summary.py` validates schema and required fields.
- Systemd unit template added: `robot/ros_ws/src/kilo_core/systemd/kilo7-perception-summary.service`.
- Interface contract updated to reflect topic name `perception_json`.

Impact:
- Provides truth-only perception summary for UI/observability; prepares inputs for Phase 4 enforcement without violating single-authority model.

Verification:
- Built `kilo_core` and confirmed perception summary publishes; test PASS.

## CT-2026-01-28-RT-025 — Ops: SAF-007 operator checklist

Date: 2026-01-28
Scope: Operational documentation to enable/verify/rollback config-gated perception enforcement

Change:
- Added `docs/CHECKLIST_SAF_007.md` detailing prerequisites, config keys, restart steps, sample trigger commands, artifacts, rollback, and troubleshooting.

Impact:
- Provides field-ready, audit-friendly steps to safely toggle and validate SAF-007 without violating single-authority or offline-first guardrails.

Verification:
- Checklist linked from `docs/BUILD_LOG.md` and reviewed against implemented config keys in `kilo.yaml` and observability fields in `safety_gate.py`.


## CT-2026-01-27 — Step 1.8: control lock reasons + relay policy logs

Date: 2026-01-27
Scope: Control lock-reason precedence; relay policy observability; no schema or topic changes

Change:
- Fixed Control lock_reason precedence and gating semantics:
  - STOP latch reports STOP_REQUEST (dominates over heartbeat stale).
  - Clear-stop results in OVERRIDE_REQUIRED / GATE_BLOCKED instead of sticky STOP.
  - Heartbeat no longer traps system in HEARTBEAT_STALE when gate is blocking.
- Added relay_kill policy transition logging (KILL/RUN with reason) for Step 1.8 auditability.

Impact:
- Control lock reasons now reflect Safety Gate truth and do not mislabel OVERRIDE_REQUIRED as STOP.
- Relay policy decisions are auditable from logs without requiring topic capture.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/control_pwm.py
- robot/ros_ws/src/kilo_core/kilo_core/relay_kill.py

Verification:
- Rebuilt with colcon --merge-install.
- Restarted systemd services.
- Step 1.8 acceptance checks passed (clear-stop + heartbeat path; relay policy logs).

## CT-2026-01-27-RT-015 — Phase 2: IMU staleness deny (Safety Gate)

Date: 2026-01-27
Scope: Safety Gate component-loss detection (IMU TTL); additive-only truth fields

Change:
- Added `safety.imu_ttl_ms` config (defaulted in config to 2000ms).
- Safety Gate now subscribes to `/kilo/phone/imu_json`, tracks receipt time, and publishes:
  - `imu_ok` and `imu_age_ms` in `/kilo/state/safety_json` (additive).
- When IMU is stale beyond `imu_ttl_ms`, Safety Gate denies with `reason="COMPONENT_MISSING"`.
- Added Step 2.3 IMU staleness integration test.

Impact:
- Robot truth explicitly reflects phone IMU staleness; Safety Gate denies motion on stale IMU.
- Control continues to clamp throttle to 0.0 on deny.

Files changed (repo):
- robot/ros_ws/src/kilo_core/config/kilo.yaml
- robot/ros_ws/src/kilo_core/kilo_core/safety_gate.py
- robot/test_step_2_3_imu_stale.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_step_1_6_invariants.py` → PASS (6/6)
- `python3 robot/test_step_1_8_ui_truth.py` → PASS (3/3)
- `python3 robot/test_step_2_3_imu_stale.py` → PASS (after stopping mqtt_bridge to force stale)

## CT-2026-01-27-RT-016 — Phase 2: Rollover deny + recovery (Safety Gate)

Date: 2026-01-27
Scope: Safety Gate rollover detection; additive-only fields

Change:
- Added `safety.rollover_tilt_deg` and `safety.rollover_hysteresis_deg` config.
- Safety Gate computes IMU tilt and latches `ROLLOVER` when over threshold; clears with hysteresis.
- Added tilt/rollover fields in safety truth (`imu_tilt_deg`, `rollover_latched`).
- Added Step 2.4 rollover integration test.

Impact:
- Safety Gate denies motion on rollover and recovers when tilt returns below threshold.
- Control continues to clamp throttle to 0.0 when denied.

Files changed (repo):
- robot/ros_ws/src/kilo_core/config/kilo.yaml
- robot/ros_ws/src/kilo_core/kilo_core/safety_gate.py
- robot/test_step_2_4_rollover.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_step_1_6_invariants.py` → PASS (6/6)
- `python3 robot/test_step_1_8_ui_truth.py` → PASS (3/3)
- `python3 robot/test_step_2_4_rollover.py` → PASS (mqtt_bridge stopped to isolate IMU)

## CT-2026-01-27-RT-017 — Phase 2: Impact deny + recovery (Safety Gate)

Date: 2026-01-27
Scope: Safety Gate impact detection; additive-only fields

Change:
- Added `safety.impact_accel_g_threshold` and `safety.impact_hysteresis_g` config.
- Safety Gate uses optional IMU `accel` field to detect impact and latches `IMPACT`.
- Added impact fields in safety truth (`imu_accel_g`, `impact_latched`).
- Added Step 2.5 impact integration test.

Impact:
- Safety Gate denies motion on impact and recovers when acceleration drops below threshold minus hysteresis.
- Control continues to clamp throttle to 0.0 on deny.

Files changed (repo):
- robot/ros_ws/src/kilo_core/config/kilo.yaml
- robot/ros_ws/src/kilo_core/kilo_core/safety_gate.py
- robot/test_step_2_5_impact.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_step_1_6_invariants.py` → PASS (6/6)
- `python3 robot/test_step_1_8_ui_truth.py` → PASS (3/3)
- `python3 robot/test_step_2_3_imu_stale.py` → PASS
- `python3 robot/test_step_2_4_rollover.py` → PASS
- `python3 robot/test_step_2_5_impact.py` → PASS

## CT-2026-01-27-RT-018 — Phase 4: Safety model publisher + calc harness

Date: 2026-01-27
Scope: Speed-aware safety model truth + calc tests (no enforcement changes)

Change:
- Added `/kilo/state/safety_model` publisher and schema `state_safety_model_v1`.
- Added safety model config profiles (crawl/normal/sport) with reaction/decel/buffer/max speed.
- Added calc-only test harness for stop-distance scaling.
- Added systemd unit for safety model node.

Impact:
- Provides deterministic, observable safety model truth for Phase 4.
- No changes to Safety Gate enforcement in this step.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/safety_model.py
- robot/ros_ws/src/kilo_core/config/kilo.yaml
- robot/ros_ws/src/kilo_core/systemd/kilo7-safety-model.service
- robot/ros_ws/src/kilo_core/setup.py
- robot/test_phase4_safety_model.py
- docs/INTERFACE_CONTRACT.md
- docs/DECISIONS_LEDGER.md

Verification:
- `python3 robot/test_phase4_safety_model.py` → PASS

## CT-2026-01-28-RT-020 — Phase 3 perception contract + stub updates

Date: 2026-01-28
Scope: Perception summary contract + stub publisher (truth-only)

Change:
- Added additive hazard/validity fields to `state_perception_v1`.
- Stub publisher now emits `stale`, `stale_reason`, `hazard_level`, `hazard_reason`,
  `min_stop_distance_m`, and per-source `age_ms`.

Impact:
- Perception truth fields are explicit and ready for Phase 3 integration.
- No Safety Gate enforcement changes in this step.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/perception_summary.py
- docs/INTERFACE_CONTRACT.md

Verification:
- `ros2 topic echo /kilo/state/perception_json --once` shows new additive fields

## CT-2026-01-27-RT-014 — Cleanup: archive old install + venv purge

Date: 2026-01-27
Scope: Maintenance / hygiene (no code behavior change)

Change:
- Archived and removed deprecated install `'/opt/kilo7_old_20260119_143131'`.
- Archived and removed Python virtual environments at `'/opt/kilo7/.venv'` and `'/opt/kilo_safety_gate/venv'`.
- Updated `.gitignore` to ignore repo-wide Python venvs and caches.

Impact:
- Eliminates confusion from duplicate installs.
- Prevents local venvs from being committed; reduces environment drift.

Files changed (repo):
- .gitignore
- docs/PROJECT_STATE.md (status updated)

Verification:
- Archives present under `/opt/kilo7/logs/cleanup-20260127-103113`:
  - `kilo7_old_20260119_143131.tar.gz`
  - `kilo7-dot-venv.tar.gz`
  - `kilo_safety_gate-venv.tar.gz`
- `systemctl daemon-reload` executed.

## CT-2026-01-26 — Step 1.8: STOP dominates heartbeat stale in control

Date: 2026-01-26
Scope: Control lock-reason precedence; no schema or topic changes

Change:
- Updated Control PWM lock-reason precedence so Safety Gate latched STOP (EXPLICIT_STOP) dominates HEARTBEAT_STALE.
- Control now caches Safety Gate latch state and reason to enforce STOP-first logic.
- Clear-stop path remains `/kilo/cmd/clear_stop_json` (cmd_clear_stop_v1).

Impact:
- Prevents HEARTBEAT_STALE from masking STOP_REQUEST when Safety Gate is latched.
- Improves control truth consistency during STOP scenarios.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/control_pwm.py

Verification:
- Step 1.8 acceptance checks A–D (STOP → CLEAR_STOP → HEARTBEAT; relay policy).

## CT-2026-01-26-RT-010 — Add explicit clear-stop command (Option B)

Date: 2026-01-26
Scope: Additive command + bridge mapping; no changes to truth topics

Problem:
- STOP latch is intentionally hard-latched. Prior behavior allowed optional clearance via UNLOCK only when `allow_clear_while_override=true`.
- Field operations require an explicit, auditable operator action to clear the STOP latch independent of UNLOCK semantics.

Change:
- Added new MQTT request topic `kilo/cmd/clear_stop` with schema `cmd_clear_stop_v1`.
- Bridged to ROS topic `/kilo/cmd/clear_stop_json`.
- Safety Gate subscribes and clears `EXPLICIT_STOP` latch on valid `cmd_clear_stop_v1`.
- `override_required` semantics remain unchanged: if `override_required=true` and override is not asserted, Safety Gate publishes `OVERRIDE_REQUIRED` and `safe_to_move=false`.

Impact:
- Provides explicit, audit-friendly latch clear separate from UNLOCK.
- Does not alter authority or truth topic schemas; strictly additive.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/safety_gate.py (subscribe + handler)
- robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py (MQTT mapping + schema enforcement)
- docs/INTERFACE_CONTRACT.md (additive reserved topic entry)
- docs/AUTHORITY_CHAIN.md (request topic listing updated)
- tools/step_1_7_verify.sh (unlock semantics detection retained; clear-stop test added)

Verification:
- Publish STOP then CLEAR_STOP on ROS:
  - `ros2 topic pub -1 /kilo/cmd/stop_json std_msgs/msg/String "{data: '{\"schema_version\":\"cmd_stop_v1\",\"ts_ms\":$(date +%s%3N)}'}"`
  - `ros2 topic pub -1 /kilo/cmd/clear_stop_json std_msgs/msg/String "{data: '{\"schema_version\":\"cmd_clear_stop_v1\",\"ts_ms\":$(date +%s%3N)}'}"`
  - Expect `/kilo/state/safety_json` to transition `latched:false`; `reason` becomes `OK` or `OVERRIDE_REQUIRED` per config.
- MQTT path: `mosquitto_pub -t kilo/cmd/clear_stop -m '{"schema_version":"cmd_clear_stop_v1","ts_ms":'"$(date +%s%3N)"'}'` and observe bridge logs `RX MQTT clear_stop` → `TX ROS /kilo/cmd/clear_stop_json`.

## CT-2026-01-26-RT-011 — Relay kill SAFE_TO_MOVE release path

Date: 2026-01-26
Scope: Safety/observability hardening (logic tweak in relay_kill)

Problem:
- Hardware relay defaults to KILL on boot (fail-closed). Control remained `locked_reason="RELAY_KILLED"` even after Safety Gate published `safe_to_move=true`, creating a bootstrap deadlock in combined Step 1.7 path.

Change:
- Update `kilo_core/relay_kill.py` to release KILL when `/kilo/state/control_json` reports `gate_safe_to_move=true` and the only lock reason is `RELAY_KILLED`.
- Sets `relay_reason="SAFE_TO_MOVE_RELEASE"` when asserting RUN via this path. Preserves existing behavior when control explicitly publishes `relay_killed=false`.

Impact:
- Breaks the final blocker in Step 1.7 by allowing relay to open once Safety Gate permits motion, without introducing a new motion authority.
- Observability improved via clear `relay_reason` values.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/relay_kill.py

Verification:
- Rebuild and restart relay service:
  - `colcon build --packages-select kilo_core`
  - `systemctl restart kilo7-relay-kill`
- Run combined path script and confirm control transitions out of `RELAY_KILLED` when gate is OK.

## CT-2026-01-26-RT-012 — Step 1.7 end-to-end verifier artifacts

Date: 2026-01-26
Scope: Tooling additions + logs captured

Change:
- Added helper `/opt/kilo7/tools/echo_json_once.py` to capture full JSON from std_msgs/String topics for reliable verification.
- Captured combined path reruns under `/opt/kilo7/logs/step_1_7/` including `final_path_rerun.txt`, `relay_status_json.txt`, `control_json_after_path.txt`.

Impact:
- Improves observability and makes verification concise and reproducible.

Verification:
- Use helper to echo topics:
  - `python3 /opt/kilo7/tools/echo_json_once.py /kilo/state/safety_json`
  - `python3 /opt/kilo7/tools/echo_json_once.py /kilo/state/control_json`
  - `python3 /opt/kilo7/tools/echo_json_once.py /kilo/hw/relay_status_json`

## CT-2026-01-26-RT-013 — Step 1.8 prep: UI truth derivation

Date: 2026-01-26
Scope: Add tooling and guidance for UI truth mapping

Change:
- Added `tools/ui_truth_probe.py` to derive `ui_lock`, `ui_emotion`, `ui_motion_allowed` from Safety/Control truth only.
- Added UI Emotion & Lockout guidance to `docs/INTERFACE_CONTRACT.md`.
- Added `tools/step_1_8_soak.sh` to record derived UI truth for soak runs.

Impact:
- Locks UI semantics to authoritative topics; prepares soak validation for Step 1.8.

Verification:
- One-shot probe: `python3 /opt/kilo7/tools/ui_truth_probe.py --once`
- Test harness: `python3 /opt/kilo7/robot/test_step_1_8_ui_truth.py`
- Soak run: `bash /opt/kilo7/tools/step_1_8_soak.sh`

## CT-2026-01-23-RT-008 — Rate-limit repetitive MQTT error alerts

Date: 2026-01-23
Scope: Observability stability (additive-only)

Problem:
- Misconfigured publishers can generate many repeated MQTT errors (schema mismatch, invalid payloads), causing alert log spam and making real issues harder to spot.

Change:
- Added simple rate limiting in `mqtt_bridge` for internally generated alerts on inbound MQTT errors (type+topic keyed). Defaults to 2s interval, configurable via `mqtt.alert_min_interval_s`.

Impact:
- Reduces noisy, duplicate alerts without changing any contract enforcement (messages are still dropped, TTL is not refreshed).
- Preserves authoritative alerts from other ROS nodes (no rate limiting applied to `/kilo/alerts_json` fanout).

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py
- robot/ros_ws/src/kilo_core/config/kilo.yaml (optional config entry)

Verification:
- Publish repeated wrong-schema `kilo/cmd/intent` at high frequency and confirm alerts are emitted at most once per interval while drops continue.
- Adjust interval via config and restart bridge to validate behavior.

## CT-2026-01-23-RT-009 — Heartbeat QoS hardening + graceful bridge shutdown

Date: 2026-01-23
Scope: Reliability hardening (transport + lifecycle) for Step 1.7

Problem:
- Occasional missed heartbeats over MQTT could contribute to control reporting `HEARTBEAT_STALE` under test conditions.
- Bridge service restart sometimes produced rclpy invalid-handle/wait-set errors due to MQTT network loop still running during ROS teardown.

Change:
- Subscribed to `kilo/cmd/heartbeat` at QoS 1 in `mqtt_bridge`; forward to ROS `/kilo/cmd/heartbeat_json` unchanged except additive `rx_ts_ms`.
- On shutdown, stop the MQTT loop and disconnect the client before destroying the ROS node and shutting down rclpy.

Impact:
- Improves delivery reliability for heartbeat messages without altering contract semantics.
- Eliminates shutdown race that could produce spurious errors on service restarts; cleaner unit lifecycle under systemd.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py

Verification:
- Manual probe: publish one `cmd_heartbeat_v1` to `kilo/cmd/heartbeat` (QoS 1) and confirm ROS echo on `/kilo/cmd/heartbeat_json`.
- Restart `kilo7-mqtt-bridge` and confirm clean logs: MQTT connected and subscriptions active; no rclpy handle/wait-set shutdown errors.

## CT-2026-01-23-RT-007 — MQTT bridge: schema-mismatch alerts + Step 1.7 verifier

Date: 2026-01-23
Scope: Additive observability for Step 1.7 + tooling

Problem:
- Critical inbound topics (intent, IMU, etc.) were dropped on schema mismatch as designed, but without alerting. Step 1.7 requires “no silent drops” for verification and UI troubleshooting.

Change:
- `mqtt_bridge` now emits a `kilo/alerts` message when a critical-topic schema mismatch occurs (still drops and does not refresh TTL).
- Added consolidated verification script to capture Step 1.7 artifacts to logs: `/opt/kilo7/tools/step_1_7_verify.sh`.

Impact:
- Improves observability for schema errors without altering contracts or enforcement behavior.
- Speeds up field verification by collecting MQTT and ROS truth outputs into timestamped logs.

Files changed (repo):
- robot/ros_ws/src/kilo_core/kilo_core/mqtt_bridge.py
- tools/step_1_7_verify.sh (new)

Verification:
- Build + restart bridge, then publish a wrong-schema intent and confirm alert:
  - `colcon build --packages-select kilo_core`
  - `systemctl restart kilo7-mqtt-bridge`
  - `mosquitto_pub -t kilo/cmd/intent -m '{"schema_version":"WRONG_v1","ts_ms":$TS,"intent":"STOP"}'`
  - Expect alert on `kilo/alerts` with `schema_mismatch`.
- Run consolidated verifier and inspect `/opt/kilo7/logs/step_1_7/*`.

Notes:
- Behavior of TTL refresh and deny/lock semantics remains unchanged; this is strictly additive alerting.

## CT-2026-01-22-RT-006 — Interface contract: Intent cadence and QoS guidance

Date: 2026-01-22
Scope: Documentation update (additive) to INTERFACE_CONTRACT — no schema or topic changes

Change:
- Added explicit guidance for `kilo/cmd/intent` publication cadence (event-driven, QoS 1, no retain, optional single retry, debounce) and STOP one-shot behavior.
- Documented heartbeat cadence (2–4 Hz) and IMU stream rate (~4 Hz) with QoS/retain rules.
- Introduced `kilo/cmd/intent` as an additive contract entry under Phone → Robot topics for Step 1.7.

Impact:
- Clarifies producer behavior and QoS/retain expectations without altering locked interfaces.
- Reduces ambiguity for app implementers and test harnesses.

Files changed (repo):
- docs/INTERFACE_CONTRACT.md

Verification:
- Rendered docs show the new "INTENT & HEARTBEAT CADENCE" section and the `kilo/cmd/intent` topic entry.

## CT-2026-01-22-RT-005 — MQTT bridge uses kilo-dev.local (mDNS)

Date: 2026-01-22
Scope: Runtime configuration alignment for Step 1.7 testing (no schema changes)

Problem:
- `mqtt_bridge` was connecting to `127.0.0.1`, while phone/app tests used the LAN broker via mDNS/IP.
- This split view prevented end-to-end IMU visibility in ROS despite broker receipt.

Change:
- Updated MQTT host in config to `kilo-dev.local` (advertised via Avahi `_mqtt._tcp`).

Impact:
- Aligns bridge with the same broker used by phone/app and CLI tests.
- Enables end-to-end verification for `cmd_intent_v1` and `phone_imu_v1`.

Files changed (repo):
- robot/ros_ws/src/kilo_core/config/kilo.yaml

Verification:
- Pending service restart: `systemctl restart kilo7-mqtt-bridge`.
- Expect bridge logs to show `host=kilo-dev.local` and subscriptions to `kilo/cmd/intent` and `kilo/phone/imu`.
- IMU test: publish valid `phone_imu_v1` to broker and observe `/kilo/phone/imu_json` in ROS.

## CT-2026-01-20-RT-002 � Enforce install-only runtime + env sanitizer; avoid ros2run fragility

Date: 2026-01-20
Scope: Runtime execution semantics + service launch robustness (no topic/schema changes intended)

Problem:
- Backend services failed under systemd due to `ros2 run` resolving entry points via dist metadata in a fragile environment.
- Workspace built with `--symlink-install` produced `.egg-link` into `build/`, making imports depend on build overlay leakage.

Change:
- Added environment sanitizer script to strip `ros_ws/build` paths from runtime env vars.
- Rebuilt workspace using `colcon build --merge-install` to ensure real installed package (no egg-link).
- Updated launch flow to run modules directly (`python3 -m kilo_core.<module>`) instead of `ros2 run`.

Impact:
- Stabilizes backend imports under systemd.
- Enforces �install overlay only� semantics at runtime.
- Reduces reliance on dist metadata lookup for service startup.
- No ROS topics/schemas/authority semantics changed by this ticket.

Files changed (repo):
- robot/ros_ws/run/kilo7-sanitize-env.sh (new)
- robot/ros_ws/run/kilo7-control.sh (updated to source sanitizer / align runtime)
- kilo7-backend.env (updated as needed to support runtime consistency)
- robot/ros_ws/src/kilo_core/kilo_core/relay_kill.py (fix import/runtime path handling as required)

Verification:
- `python3 -c "import kilo_core.relay_kill as m; print(m.__file__)"`
  returns install site-packages path.
- `systemctl is-active kilo7-relay-kill kilo7-safety-gate kilo7-mqtt-bridge kilo7-control` => active
- `systemctl show <unit> -p ExecStart` confirms `python3 -m ...` usage.

---

## CT-2026-01-20-RT-003 � Stop tracking ROS build/install/log + python artifacts; tighten gitignore

Date: 2026-01-20
Scope: Repo hygiene / reproducibility (no runtime behavior change intended)

Change:
- `.gitignore` updated to permanently exclude:
  - robot/ros_ws/build/
  - robot/ros_ws/install/
  - robot/ros_ws/log/
  - python caches/artifacts (__pycache__, *.pyc, *.pyo, *.pyd, *.egg-info, *.egg-link)
  - common tool caches (.pytest_cache, .mypy_cache, .ruff_cache, .cache)

Impact:
- Prevents accidental commits of machine-local build products.
- Reduces �it works on my machine� drift and review noise.

Files changed:
- .gitignore

Verification:
- `git ls-files | grep -E '^robot/ros_ws/(build|install|log)/'` => none

---

## CT-2026-01-20-RT-004 � Canonical systemd units in repo + apply script

Date: 2026-01-20
Scope: Deployment consistency (no topic/schema changes intended)

Change:
- Added canonical unit files into repo:
  - robot/ros_ws/run/systemd/kilo7-relay-kill.service
  - robot/ros_ws/run/systemd/kilo7-safety-gate.service
  - robot/ros_ws/run/systemd/kilo7-mqtt-bridge.service
  - robot/ros_ws/run/systemd/kilo7-control.service
- Added apply script:
  - robot/ros_ws/run/systemd/apply-kilo7-systemd.sh
- Units:
  - source ROS setup + workspace install setup
  - source sanitizer
  - run `python3 -m kilo_core.<module>`
  - restart=always, restartsec=1

Impact:
- Makes systemd deployment reproducible from git.
- Eliminates host-only unit drift (single source of truth = repo).

Verification:
- Apply script installs units to `/etc/systemd/system`, daemon-reload, enable, restart.
- `systemctl show <units> -p FragmentPath` points to `/etc/systemd/system/<unit>.service`
- `systemctl is-active <units>` => active, `NRestarts=0` after restart.

---
# Changelog

## 2026-01-19 � Repo-on-Robot Baseline (Option A) + Safe Auto-Pull

### Summary
Established the robot Pi as an authoritative git clone host at `/opt/kilo7` (Option A) and enabled safe automatic repo updates via a pull-only systemd timer.

### Changes Made
- `/opt/kilo7` is now a real git clone tracked to `origin/main`
- Added safe auto-pull tool:
  - `tools/kilo7_git_autopull.sh` (ff-only, refuses when working tree is dirty)
- Added systemd pull automation:
  - `kilo7-git-autopull.service` + `kilo7-git-autopull.timer`
  - `SuccessExitStatus=3` so �dirty tree refusal� is non-fatal (prevents log spam)

### Verification
- `git rev-parse HEAD` returns a commit hash
- `git status --porcelain` clean during auto-pull success
- Auto-pull logs show �Already up to date� when no updates exist

### Result
Guardrails and state validation can function again because runtime and docs now derive from a verifiable repo state on the robot.

---

## 2026-01-19 � Safety Authority Unification / One-Path Enforcement

### Summary
Eliminated a hidden dual-safety-authority condition that violated the �one path to safety� rule. A legacy HTTP-based Safety Gate service (`kilo-safety-gate.service`) could exist alongside the ROS-based Safety Gate (`kilo7-safety-gate.service`), creating potential for forked safety truth and undefined behavior.

### Problem
Two independent safety mechanisms could exist simultaneously:
- **ROS Safety Gate** (`kilo7-safety-gate.service`) publishing authoritative safety truth on `/kilo/state/safety_json`
- **Legacy HTTP Safety Gate** (`kilo-safety-gate.service`) exposing `/safety/*` endpoints on TCP 8098

Even when �disabled,� the legacy unit could be restarted, reintroducing a second safety authority and violating the single-authoritative-path requirement.

### Changes Made
- Fully **disabled and masked** the legacy `kilo-safety-gate.service`
- Removed the unit file and override directory
- Replaced the unit with a **symlink to `/dev/null`** to guarantee it cannot be started or re-enabled
- Reloaded systemd to enforce masking

### Verification
- `systemctl is-enabled kilo-safety-gate.service` ? **masked**
- `systemctl status kilo-safety-gate.service` ? **inactive (dead), masked**
- `ss -ltnp | grep :8098` ? **no listeners**

### Result
- Exactly **one** Safety Gate exists at runtime
- All safety truth flows through a single authoritative chain:
  **Command ? Safety Gate (ROS) ? Control ? Actuators ? State**
- Impossible to regress into a dual-safety-authority condition without deliberate manual reversal

### Status
? Closed

---

## 2026-01-19 � Backend Stabilization � MQTT Bridge UTF-8 Crash Fix

### Fixed
- Resolved fatal UTF-8 decode error in `kilo_core/mqtt_bridge.py` caused by a non-UTF8 cp1252 character (0x96, Windows en-dash) in a docstring.
- Rebuilt ROS workspace to propagate corrected UTF-8 source into `build/` and `install/` overlays.
- Fixed `kilo7-mqtt-bridge.service` ExecStart quoting and ROS environment sourcing so `ros2 run` launches reliably under systemd.
- Prevented `kilo7-control` startup failure due to unset optional ROS environment variables (observed under systemd).

### Changed
- `kilo7-mqtt-bridge.service`: replaced fragile inline ExecStart with a shell-based invocation that explicitly sources ROS and workspace environments before execution.
- `run/kilo7-control.sh`: removed `set -u` and defensively initialized `AMENT_TRACE_SETUP_FILES` to avoid abort on unset variables.

### Verified
- MQTT ? ROS bridge remains active and publishes state topics at expected cadence (no dropouts observed during validation window).
- `/kilo/state/control` publishes live (non-retained) messages.
- `/kilo/state/safety_json` and `/kilo/state/control_json` conform to the defined interface contract.
- MQTT command topics observed and bridged as designed (no schema or topic changes introduced).

### Known Issues
- Safety Gate correctly enforces `OVERRIDE_REQUIRED` and `EXPLICIT_STOP`.
- A mismatch was observed between control state and safety state during diagnosis.

#### Impact
- This mismatch can mislead downstream consumers or UI into believing motion is permitted when it is not.
- Violates the invariant that **Safety Gate is the sole motion authority** if consumers treat control state as authoritative.

#### Status
- Diagnosed only.
- No behavioral changes implemented in this change set.

---

## 2026-01-17 � CT-2026-01-17-RT-001 � Hotfix: backend runtime + repeatability

### Fixed
- Installed MQTT bridge dependency: `python3-paho-mqtt` (prevents `ModuleNotFoundError: No module named 'paho'`).
- Patched systemd units (`kilo7-control`, `kilo7-safety-gate`, `kilo7-relay-kill`, `kilo7-mqtt-bridge`) to avoid invalid ROS CLI arg `-p config:=` when `KILO_CONFIG` is unset.
  - Units now only pass `--ros-args -p config:=...` when `KILO_CONFIG` is non-empty.
- Restored environment files for deterministic startup:
  - `/etc/default/kilo7`
  - `/opt/kilo7/kilo7-backend.env`

### Verified
- All services run without restart loops.
- MQTT truth topics publish valid JSON: `kilo/health`, `kilo/state/safety`, `kilo/state/control`.

---

## 2026-01-16 � Backend Build Plan Alignment (ROS 2 Prerequisite Gate)

Type: Process / Build-plan correction  
Scope: Documentation + build sequencing only  
Code impact: None (backend code unchanged)  
Contract impact: None (INTERFACE_CONTRACT v1.2 remains sealed)

### Summary
Aligned the build plan and install flow with the finalized KILO .7 backend package, which is a ROS 2 workspace and therefore requires ROS 2 to be installed before backend installation and build.

This change corrects an implicit assumption in earlier planning that backend installation could precede ROS 2 setup.

### What Changed
- Introduced an explicit Phase 0: ROS 2 Prerequisite Gate in the build plan.
- Clarified that:
  - The backend bundle is not a standalone runtime.
  - `colcon build`, `rclpy`, and ROS environment sourcing are mandatory before backend build/install steps.

### What Did NOT Change
- Backend code, nodes, or systemd units
- Safety Gate semantics
- Control loop behavior (TTL, lock, heartbeat, relay enforcement)
- MQTT topic contracts or schemas
- Deployment layout assumptions (`/opt/kilo7`, `kilo` user)

### Rationale
The backend README and install package already enforce ROS 2 usage (via colcon, rclpy, and ROS launch expectations). The build plan was updated to match reality, reduce install failures, and prevent �winging it� during setup.

### Impact on Future Builds
All future backend installs must pass the ROS 2 gate before proceeding.

### Risk Assessment
Low risk: documentation-only correction  
High value: prevents invalid installs and wasted debug time

### Approval / Notes
This change restores alignment between backend README, install scripts, systemd assumptions, and build execution order.
