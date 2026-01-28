# Build Log — Phase 2 Status

Date: 2026-01-27

## Summary
- Impact detection implemented in Safety Gate with hysteresis; rollover test refined to be self-contained with bridge running.
- All Step 1.6/1.8 and Phase 2 tests PASS; acceptance checks satisfied.
- Rosbridge websocket is running (9090 in use) to support visualization.

## Changes
- Safety Gate impact detection:
  - Config: `safety.impact_accel_g_threshold: 2.0`, `safety.impact_hysteresis_g: 0.3`
  - Parses optional IMU accel field and latches IMPACT
  - Publishes additive fields `imu_accel_g`, `impact_latched`
- Rollover test self-contained:
  - Waits for `imu_ok` and publishes IMU via ROS within test
- New test: `robot/test_step_2_5_impact.py`
- Docs updated: change log, interface contract, decisions

## Verification
- `robot/test_step_1_6_invariants.py` → PASS (6/6)
- `robot/test_step_1_8_ui_truth.py` → PASS (3/3)
- `robot/test_step_2_3_imu_stale.py` → PASS
- `robot/test_step_2_4_rollover.py` → PASS
- `robot/test_step_2_5_impact.py` → PASS
- Logs: `logs/phase_2/20260127-152922-impact/`

## Acceptance Checks
- Single publisher on safety/control topics ✔
- IMU stream present ✔
- Rollover deny + clamp + recovery ✔
- Impact deny + clamp + recovery ✔
- Existing suites still pass ✔

## Commit
- 27b629c — "CT-2026-01-27-RT-017: impact deny + test; refine rollover test" (pushed)

## Notes
- Rosbridge: Port 9090 is occupied (expected if service active). Visualizer can connect via `ws://kilo-dev.local:9090`.
- Offline-first guardrails: No cloud dependencies added; single motion authority preserved.

## Next Options
- Speed-aware safety model: implement velocity profiles (crawl/normal/sport) and stop distance scaling; add tests.
- Observability: add gate decision latency metrics to `state_safety_json` for field tuning.
- Soak: run extended impact/rollover soak tests and archive artifacts.

---

Date: 2026-01-28

## Phase 4 — Safety Model (truth publishing)
- Implemented `kilo_core.safety_model` node publishing `/kilo/state/safety_model` (schema: `state_safety_model_v1`).
- Added systemd unit `kilo7-safety-model.service` and enabled it.
- Configured profiles in `kilo.yaml` (crawl/normal/sport) with reaction/brake/buffer/max_speed.
- Calculation-only test aligned to config: `robot/test_phase4_safety_model.py` → PASS.
- Contracts and decisions updated (additive-only) to reflect published model.

Important: Enforcement not wired into Safety Gate yet (by design). Phase 3 perception must feed hazard/stop metrics before gating; publishing truth now enables UI/observability and future enforcement without violating single-authority rules.

Verification
- `/kilo/state/safety_model` publishes with profile "normal" and valid outputs.
- Logs: `logs/phase_4/20260128-080551-safety-model/` (build + test outputs).

Commit
- 517f11b — "CT-2026-01-28-RT-019: Phase 4 safety model implementation" (pushed)

Next Options
- Capture one-shot of `/kilo/state/safety_model` to logs for audit.
- Proceed to Phase 3 perception contracts to enable Safety Gate enforcement wiring.

---

Date: 2026-01-28

## Health + One-Shot Captures (post SAF-007)
- Ran systemd health check for `kilo7-*` services (safety-gate, control, relay-kill, mqtt-bridge, safety-model, perception-summary, rosbridge).
- Captured one-shots of truth topics for audit using helper scripts:
  - Safety: `tools/capture_safety_once.sh` → logs saved under `logs/phase_4/<ts>-safety-once/safety_once.json`
  - Perception: `tools/capture_perception_once.sh` → logs saved under `logs/phase_3/<ts>-perception-once/perception_once.json`

Artifacts (exact paths)
- [logs/phase_4/20260128-115603-safety-once/safety_once.json](logs/phase_4/20260128-115603-safety-once/safety_once.json)
- [logs/phase_3/20260128-115603-perception-once/perception_once.json](logs/phase_3/20260128-115603-perception-once/perception_once.json)
- SAF-007 PASS run: [logs/phase_4/20260128-113403-saf-007-pass/](logs/phase_4/20260128-113403-saf-007-pass/)

Operator Guide
- See [docs/CHECKLIST_SAF_007.md](docs/CHECKLIST_SAF_007.md) for enable/verify/rollback steps and sample triggers.

Reproduce:
```bash
systemctl --no-pager --type=service | grep -E 'kilo7-(safety-gate|control|relay-kill|mqtt-bridge|safety-model|perception-summary|rosbridge)'
bash /opt/kilo7/tools/capture_safety_once.sh
bash /opt/kilo7/tools/capture_perception_once.sh
```

Notes:
- Captures reflect current config-gated enforcement (SAF-007). Enablement remains off by default.

---

Date: 2026-01-28

## Phase 3 — Perception Summary (truth-only stub)
- Implemented `kilo_core.perception_summary` publishing `/kilo/state/perception_json` (`state_perception_v1`).
- Added systemd unit template `kilo7-perception-summary.service`.
- Added test `robot/test_phase3_perception_summary.py` → validates schema/fields; aligned to contracts.
 - Added validity test `robot/test_phase3_perception_validity.py` → verifies staleness flags, per-source ages, hazard fields.

Verification
- Built `kilo_core` and executed the test; perception summary publishes at 1 Hz.
 - Validity test PASS: confirms additive fields presence and types.
- Optional logs: one-shot capture under `logs/phase_3/` via helper script.

Notes
- Stub publishes truth for observability. Enforcement wiring remains deferred until hazards/stop metrics are available.

---

Date: 2026-01-28

## Phase 4 — Perception Enforcement Wiring (config-gated)
- Implemented Safety Gate wiring to perception hazards and safety_model stop distance.
- Added config keys under `safety.perception_enforcement` (default `enabled: false`).
- Added additive safety truth fields: `perception_ok`, `perception_age_ms`, `perception_reason`, `stop_distance_m`.
- Added integration test `robot/test_phase4_enforcement_wiring.py` (toggles enabled/disabled).

Verification
- `python3 robot/test_phase4_enforcement_wiring.py` → PASS (disabled + enabled scenarios)
- Existing suites: `robot/test_step_1_6_invariants.py`, `robot/test_step_1_8_ui_truth.py` → PASS
- Logs: `logs/phase_4/20260128-113403-saf-007-pass/`

Notes
- Enforcement is opt-in by config; default behavior unchanged.
- The Phase 4 wiring test runs Safety Gate in-process and temporarily stops systemd Safety Gate and MQTT bridge; both are restored after test.

---

Date: 2026-01-28

## Phase 5 — Mapping Contracts + Stub Publisher
- Added `state_mapping_v1` contract for `/kilo/state/mapping_json`.
- Implemented `kilo_core.mapping_summary` stub publisher (truth-only, 1 Hz).
- Added test `robot/test_phase5_mapping_summary.py`.
- Added systemd unit template `kilo7-mapping-summary.service` (disabled by default).

Verification
- `python3 robot/test_phase5_mapping_summary.py` → PASS
- Logs: `logs/phase_5/20260128-123353-mapping/`

---

Date: 2026-01-28

## Phase 5 — Mapping Soak + UI Truth Probe Update
- Added `tools/phase_5_mapping_soak.sh` to capture mapping truth + UI truth for 60s.
- UI truth probe now includes mapping observability fields.

Verification
- Soak logs: `logs/phase_5/20260128-125534-mapping-soak/`

---

Date: 2026-01-28

## Phase 5 — Mapping UI One-shot Capture
- Added `tools/capture_mapping_once.sh` for mapping/UI snapshot.

Verification
- Snapshot: `logs/phase_5/20260128-132130-mapping-once/`

---

Date: 2026-01-28

## Connection Info Snapshot (Visualizer)
- Captured rosbridge connection details and service status.
- Current IP: 10.10.10.63 (from `hostname -I`)
- Recommended endpoints:
  - `ws://kilo-dev.local:9090`
  - `ws://10.10.10.63:9090`

Verification
- Logs: `logs/phase_5/20260128-132803-connection-info/`

Notes
- rosbridge service shows repeated restarts due to ROS logging directory error (`Failed to get logging directory`).

---

Date: 2026-01-28

## Phase 6 — Navigation Contracts + Stub Publisher
- Added `state_navigation_v1` contract for `/kilo/state/navigation_json`.
- Implemented `kilo_core.navigation_summary` stub publisher (truth-only, 1 Hz).
- Added test `robot/test_phase6_navigation_summary.py`.
- Added systemd unit template `kilo7-navigation-summary.service` (disabled by default).

Verification
- `python3 robot/test_phase6_navigation_summary.py` → PASS
- Logs: `logs/phase_6/20260128-140840-navigation/`

---

Date: 2026-01-28

## Phase 7 — Docking Contracts + Stub Publisher
- Added `state_docking_v1` contract for `/kilo/state/docking_json`.
- Implemented `kilo_core.docking_summary` stub publisher (truth-only, 1 Hz).
- Added test `robot/test_phase7_docking_summary.py`.
- Added systemd unit template `kilo7-docking-summary.service` (disabled by default).

Verification
- `python3 robot/test_phase7_docking_summary.py` → PASS
- Logs: `logs/phase_7/20260128-144656-docking/`

---

Date: 2026-01-28

## Verification Suite — Full Pass
- Ran full verification suite across phases 1–7 (truth-only stubs + safety gating).
- Fixed runner to avoid ROS setup unbound variable; hardened IMU stale test against explicit_stop latch.

Verification
- Summary: `logs/phase_6/20260128-152421-verification/summary.txt`
- Latest full suite: `logs/phase_6/20260128-154159-verification/summary.txt`

---

Date: 2026-01-28

## Lockdown Status
- Phases 1–7 contracts + stubs verified; full suite PASS.
- Remaining ops gaps: rosbridge stability, MQTT load/soak, multi-node failure recovery.
- Ops hardening runs complete:
  - rosbridge logging dir fix applied; service stable on 9090.
  - MQTT load/soak captured.
  - Core service recovery check captured.

Verification
- MQTT soak: `logs/ops/20260128-154927-mqtt-soak/`
- Recovery check: `logs/ops/20260128-155146-recovery/`
