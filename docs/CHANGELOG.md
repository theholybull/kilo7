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

