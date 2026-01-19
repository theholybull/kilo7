## [2026-01-19] Backend Stabilization — MQTT Bridge UTF-8 Crash Fix

### Fixed
- Resolved fatal UTF-8 decode error in `kilo_core/mqtt_bridge.py` caused by a non-UTF8 cp1252 character (0x96, Windows en-dash) in a docstring.
- Rebuilt ROS workspace to propagate corrected UTF-8 source into `build/` and `install/` overlays.
- Fixed `kilo7-mqtt-bridge.service` ExecStart quoting and ROS environment sourcing so `ros2 run` launches reliably under systemd.
- Prevented `kilo7-control` startup failure due to unset optional ROS environment variables (observed under systemd).

### Changed
- `kilo7-mqtt-bridge.service`: replaced fragile inline ExecStart with a shell-based invocation that explicitly sources ROS and workspace environments before execution.
- `run/kilo7-control.sh`: removed `set -u` and defensively initialized `AMENT_TRACE_SETUP_FILES` to avoid abort on unset variables.

### Verified
- MQTT ↔ ROS bridge remains active and publishes state topics at expected cadence (no dropouts observed during validation window).
- `/kilo/state/control` publishes live (non-retained) messages.
- `/kilo/state/safety_json` and `/kilo/state/control_json` conform to the defined interface contract.
- MQTT command topics observed and bridged as designed (no schema or topic changes introduced).

### Known Issues
- Safety Gate (Step 1.4) correctly enforces `OVERRIDE_REQUIRED` and `EXPLICIT_STOP`.
- `kilo/state/control` reports `gate_safe_to_move=true` while `kilo/state/safety` reports deny.

#### Impact
- This mismatch can mislead downstream consumers or UI into believing motion is permitted when it is not.
- Violates the invariant that **Safety Gate is the sole motion authority** if consumers treat control state as authoritative.

#### Status
- Diagnosed only.
- No behavioral changes implemented in this change set.
VERSION / DATE:
2026-01-19

WHAT CHANGED:
- Step 1.4 Safety Gate brought up as a logic-only HTTP authority service (port 8098).
- Removed temporary override behavior by setting override_required=false after relay kill-path validation.
- Verified deny/allow matrix: LOSS_OF_COMMAND, EXPLICIT_STOP latch, COMPONENT_MISSING, and ALLOW/NONE.

WHY:
- Relay kill-path is implemented and validated in Step 1.3, so Safety Gate can operate without override debt.
- Establish single stop authority and explicit reasons prior to building the main backend.

STATUS:
tested (logic-only, no motion)
## 2026-01-17 — CT-2026-01-17-RT-001 — Hotfix: backend runtime + repeatability

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


CHANGE_LOG.md
2026-01-16 — Backend Build Plan Alignment (ROS 2 Prerequisite Gate)

Type: Process / Build-plan correction
Scope: Documentation + build sequencing only
Code impact: None (backend code unchanged)
Contract impact: None (INTERFACE_CONTRACTS v1.2 remains sealed)

Summary

Aligned the build plan and install flow with the finalized KILO .7 backend package, which is a ROS 2 workspace and therefore requires ROS 2 to be installed before backend installation and build.

This change corrects an implicit assumption in earlier planning that backend installation could precede ROS 2 setup.

What Changed

Introduced an explicit Phase 0: ROS 2 Prerequisite Gate in the build plan.

Clarified that:

The backend bundle is not a standalone runtime.

colcon build, rclpy, and ROS environment sourcing are mandatory before backend build/install steps.

Replaced references to updating PROJECT_STATE.md during builds with:

CHANGE_LOG.md / BUILD_LOG.md entries instead (PROJECT_STATE is immutable by design).

What Did NOT Change

Backend code, nodes, or systemd units

Safety Gate semantics

Control loop behavior (TTL, lock, heartbeat, relay enforcement)

MQTT topic contracts or schemas

Deployment layout assumptions (/opt/kilo7, kilo user)

Rationale

The backend README and install package already enforce ROS 2 usage (via colcon, rclpy, and ROS launch expectations). The build plan was updated to match reality, reduce install failures, and prevent “winging it” during setup.

This keeps:

Install steps deterministic

Failures early and obvious

Future phases (perception, mapping, navigation) aligned with the same ROS-first model

Impact on Future Builds

All future backend installs must pass the ROS 2 gate before proceeding.

Phase sequencing is now stable for Phase 3+ work.

No rework or migration is required for existing backend deployments that already have ROS 2 installed.

Risk Assessment

Low risk: documentation-only correction

High value: prevents invalid installs and wasted debug time

No regression risk to running systems

Approval / Notes

PROJECT_STATE.md intentionally unchanged

This change restores alignment between:

Backend README

Install scripts

Systemd assumptions

Build execution order

VERSION / DATE:
2026-01-15

WHAT CHANGED:
- Implemented Safety Gate v1 as logic-only HTTP service on port 8098.
- Added systemd drop-in to set PYTHONPATH for reliable imports.
- Verified acceptance checks: explicit stop latch, clear blocked under override, component fault injection, heartbeat age update.

WHY:
- Establish centralized stop authority early without permitting motion.
- Preserve explicit safety debt: relay kill-path not implemented.

STATUS:
tested
# Changelog

