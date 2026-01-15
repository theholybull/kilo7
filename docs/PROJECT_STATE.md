CURRENT PHASE:
Phase 1 — Bring-up

CURRENT GOAL:
Step 1.3 — GPIO, PWM, relay kill-path validation

LAST CONFIRMED WORKING STATE:
- GPIO control verified via gpiod CLI:
  - /dev/gpiochip0 present; gpioset gpiochip0 17=1/0 toggles attached device power indicator reliably.
- PCA9685 PWM controller verified on I2C bus:
  - i2cdetect -y 1 shows PCA9685 at 0x40 (device at 0x70 present but untouched).
  - Python (Blinka + adafruit_pca9685) works.
  - PCA9685 set to 50 Hz; channel commands succeed.
  - “All channels off” sets duty_cycle=0 for all 16 channels deterministically.
  - Post-reboot: i2cdetect still sees 0x40 and “all channels off” still works.

WHAT IS BROKEN (KNOWN):
- None at this step.

WHAT IS UNTESTED:
- True hardware relay kill-path inline with ESC/motor power (fail-closed independent of software). Current GPIO17 power-gating of PWM board VCC is a temporary test and not an acceptable final kill-path.

NEXT CONCRETE STEP:
Step 1.4 — Safety Gate bring-up, AFTER implementing and validating a real relay kill-path inline with ESC power.

CURRENT PHASE:
Phase 1 — Bring-up

CURRENT GOAL:
Step 1.4 — Safety Gate Bring-up (logic + interfaces only, motion locked)

LAST CONFIRMED WORKING STATE:
- Safety Gate service running and listening on 0.0.0.0:8098
- Endpoints verified:
  - GET /health returns {"ok":true,"service":"safety_gate_v1"}
  - GET /safety/gate returns gate snapshot with explicit fields
  - POST /safety/stop latches explicit_stop=true
  - POST /safety/clear returns cleared=false while override_required=true (intentional)
  - POST /safety/component sets component_ok[pwm]=false
  - POST /safety/heartbeat updates heartbeat_age_s (non-null)
- Gate fails closed with reason OVERRIDE_REQUIRED (expected due to safety debt)
- systemd import-path fixed via drop-in:
  - /etc/systemd/system/kilo-safety-gate.service.d/override.conf
  - PYTHONPATH=/opt/kilo_safety_gate
  - PYTHONUNBUFFERED=1

WHAT IS BROKEN (KNOWN):
- Real fail-closed relay kill-path inline with ESC power is NOT implemented (KNOWN SAFETY DEBT)
- Motion remains prohibited in Step 1.4 (by scope)

WHAT IS UNTESTED:
- Integration point where the main control loop consumes gate.allow (arbitration seam)
- Any hardware enforcement (relay, PWM neutralization, etc.) — out of scope for Step 1.4

NEXT CONCRETE STEP:
- Define and implement the single arbitration seam in the main control stack where gate.allow is required for any actuator command (still motion-locked).
- Add observability: surface gate.allow/reason in the primary system status/telemetry channel.

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


# Safety Gate (Step 1.4) — Logic-Only Authority

## Purpose
Safety Gate is the single authority that determines whether motion is permitted.
It is fail-closed by default and returns explicit deny reasons.

This Step 1.4 implementation defines logic + interfaces only.
No motion is permitted in this step.

## Service
- HTTP: `http://127.0.0.1:8098`
- systemd: `kilo-safety-gate.service`

## Output Contract
Gate emits a single canonical snapshot:

- `allow` (bool): the only motion permission
- `reason` (string enum): explicit deny reason (or NONE when allow=true)
- `ts_monotonic` (float): monotonic timestamp
- `heartbeat_age_s` (float|null): seconds since last heartbeat
- `explicit_stop` (bool): latched stop flag
- `component_ok` (object): component health flags
- `battery_ok` (bool|null): battery health

## Reasons (v1)
- BOOT
- LOSS_OF_COMMAND
- EXPLICIT_STOP
- COMPONENT_MISSING
- BATTERY_CRITICAL
- OVERRIDE_REQUIRED
- NONE (allow=true only)

## Priority Order (highest first)
1) OVERRIDE_REQUIRED (forces deny while safety debt exists)
2) EXPLICIT_STOP (latched stop)
3) LOSS_OF_COMMAND (heartbeat stale/missing)
4) COMPONENT_MISSING (any component_ok == false)
5) BATTERY_CRITICAL (battery_ok == false)
6) ALLOW (only if none of the above)

## Boot Behavior
On process start:
- gate initializes to `DENY / BOOT`
- first evaluation computes current state from inputs

## Loss-of-Command
If heartbeat is missing or older than `heartbeat_timeout_s`:
- gate denies with `LOSS_OF_COMMAND`

## Override (Safety Debt)
Because a real fail-closed relay kill-path inline with ESC power is NOT implemented:
- `override_required=true`
- gate denies with `OVERRIDE_REQUIRED` regardless of other inputs

This prevents false confidence and keeps Step 1.4 motion locked.

