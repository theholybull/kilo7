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
