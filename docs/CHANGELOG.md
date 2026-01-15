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

