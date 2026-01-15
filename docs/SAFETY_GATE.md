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
