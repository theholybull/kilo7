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
