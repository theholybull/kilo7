✅ SEALED CANONICAL DOCUMENT

Below is the complete, clean, canonical version with all guardrails locked.

INTERFACE_CONTRACTS — KILO .7

Version: v1.2
Status: CANONICAL — ADDITIVE ONLY
Last updated: 2026-01-15

This document defines the stable communication and state interfaces for KILO .7, covering Phase 1 through Phase 7.

This is a contract, not an implementation plan.

Once published, this document must not be weakened.
Changes require a Change Ticket and must remain additive-only.

NON-NEGOTIABLES

Offline-first operation.

Headless robot (phone = sensors + UI only).

ROS 2 is the robot core runtime.

MQTT is the LAN comms spine.

Safety Gate is the single motion authority.

Core control must run without perception or navigation.

Additive-only evolution: never rename or remove fields or topics.

AUTHORITY MODEL (LOCKED)
Requests vs Truth (must never blur)

kilo/cmd/* → requests (untrusted; may be denied)

kilo/ui/* → requests (untrusted; may be denied)

kilo/phone/* → untrusted phone sensor inputs

kilo/state/* → robot truth (authoritative)

Robot truth about phone data must be published explicitly in kilo/state/*
(staleness, validity, ages).
Consumers must never infer truth from phone topics directly.

Motion Authority (LOCKED)

Safety Gate publishes safe_to_move=false → robot must refuse motion

Control node is the single enforcement point for actuation

Kill relay is hardware enforcement, fail-closed on boot
(it is not a second arbiter)

TIME + STALENESS RULES (LOCKED)
Timestamp Format (MUST)

Every payload MUST include:

ts_ms: integer milliseconds since Unix epoch (UTC)

Rules:

Phone clocks may be skewed

Robot MUST NOT compute staleness from incoming ts_ms

Robot computes staleness using receipt time and/or monotonic time

Optional (allowed, not required):

ts_src: "phone_wall" | "robot_wall"

Two-TTL Model (MUST)

Staleness enforcement is split into two independent timeouts.

1) Command TTL — cmd_ttl_ms

Applies to: kilo/cmd/drive

If time since last valid drive request > cmd_ttl_ms:

throttle MUST go neutral immediately

stale_cmd=true MUST be published

Valid drive request definition (LOCKED):

Payload parses

schema_version matches

steer and throttle are within range

Invalid packets MUST NOT refresh TTL

2) Heartbeat TTL — hb_ttl_ms

Applies to: controller/UI liveness

Reserved topic (optional until implemented):

kilo/cmd/heartbeat
Schema: cmd_heartbeat_v1

If heartbeat is implemented and stale:

robot MUST enter lockout

lock reason MUST be published

Drive cadence MUST NOT be used as the only heartbeat long-term.

STOP vs DENY vs LOCK (LOCKED)
Precedence (ABSOLUTE)

Safety Gate deny
→ applied throttle neutral immediately

STOP request
→ locked=true immediately
→ applied throttle neutral immediately

LOCKED state
→ forbids motion regardless of requests

Clearing Lock (MUST)

Lock MUST NOT clear silently

Clearing lock requires explicit action

Reserved topic:

kilo/cmd/unlock
Schema: cmd_unlock_v1

Explicit STOP latch reset (additive):

Reserved topic:

kilo/cmd/clear_stop
Schema: cmd_clear_stop_v1

Robot truth MUST include:

locked_reason

(optional) unlock_allowed

Safety Gate deny and lock are independent and may coexist.

KILL RELAY CONTRACT (LOCKED)
Definitions

Safety Gate remains the only motion authority

Relay kill is hardware enforcement only

Relay kill may be asserted for CRITICAL conditions

Relay kill does not create a second arbiter

Truth Fields (MUST)

Robot truth MUST include:

relay_killed: bool

relay_reason: string (additive-only)

Published in:

kilo/state/control (recommended)

QOS + RETAIN RULES (LOCKED)

Commands and inputs (kilo/cmd/*, kilo/ui/*, kilo/phone/*)

MUST NOT be retained

Truth/state topics

SHOULD be retained (last known truth)

QoS rules:

Safety authority, lock, mode, and applied control truth → QoS 1

High-rate non-critical telemetry → QoS 0 allowed

INTENT & HEARTBEAT CADENCE (GUIDANCE — ADDITIVE)

Intent (Voice/UI) — `kilo/cmd/intent` (schema: cmd_intent_v1)

- Event-driven only (one-shot on user action); DO NOT publish periodically
- QoS 1, retain=false
- Idempotent by design; duplicates are tolerated (use `utterance_id` for correlation)
- Optional single retry: re-publish the same payload once after ~200–500 ms for robustness
- Debounce: suppress identical repeats for ~1 s unless `utterance_id` changes
- STOP intent: single publish is sufficient (Safety Gate latches `EXPLICIT_STOP`); streaming STOP is forbidden

Heartbeat — `kilo/cmd/heartbeat` (schema: cmd_heartbeat_v1)

- Maintain liveness at 2–4 Hz (≥2 Hz recommended)
- QoS 0 or 1, retain=false
- `hb_ttl_ms` defines lockout threshold; current default 1000 ms implies >1 Hz required

Phone IMU — `kilo/phone/imu` (schema: phone_imu_v1)

- Sensor stream independent of intents; typical 4 Hz is recommended
- QoS 0, retain=false
- Staleness is enforced by Safety Gate using `safety.imu_ttl_ms` (robot clock; additive-only)
- Rollover detection uses `safety.rollover_tilt_deg` with hysteresis `safety.rollover_hysteresis_deg`
- Impact detection uses optional `accel` field and `safety.impact_accel_g_threshold` with hysteresis `safety.impact_hysteresis_g`

SCHEMA EVOLUTION (DRIFT KILLER)
UI EMOTION & LOCKOUT (GUIDANCE — ADDITIVE)

UI derives emotion/lockout strictly from authoritative Pi truth topics:

- `kilo/state/safety`: `safe_to_move`, `reason`, `latched`, `override_required`
- `kilo/state/control`: `locked`, `locked_reason`, `relay_killed`, `relay_reason`, `applied.throttle`

Rules:

- Never infer emotion/lockout from request topics or transport state (e.g., MQTT connection, "command sent").
- Suggested precedence for UI emotion: Safety `reason` (when `safe_to_move=false`) → Control `locked_reason` → `RELAY_KILLED` → `OK`.
- UI lock condition: lock when `safe_to_move=false` OR `locked=true` OR `relay_killed=true`.
- Motion allowed flag (intent-level): `safe_to_move && !locked && !relay_killed`.

Reference helper:

- `/opt/kilo7/tools/ui_truth_probe.py` prints a compact JSON suitable for UI derivation and soak tests.

Every payload MUST include schema_version

Exact string match

Consumers MUST ignore unknown fields

Producers MUST NOT rename or remove fields

New fields only; safe defaults when absent

TOPIC CONTRACT SPEC
Namespace

All topics live under:

kilo/...

PHASE 1–2 REQUIRED TOPICS
Phone → Robot
IMU

kilo/phone/imu — schema phone_imu_v1
Untrusted sensor input.

GPS (optional)

kilo/phone/gps — schema phone_gps_v1

Drive Request

kilo/cmd/drive — schema cmd_drive_v1
Request only. May be denied.

STOP Request

kilo/cmd/stop — schema cmd_stop_v1

Heartbeat (reserved)

kilo/cmd/heartbeat — schema cmd_heartbeat_v1

Unlock (reserved)

kilo/cmd/unlock — schema cmd_unlock_v1

Clear Stop (reserved)

kilo/cmd/clear_stop — schema cmd_clear_stop_v1

Mode Request

kilo/ui/mode_request — schema ui_mode_request_v1

Emotion Request

kilo/ui/emotion_request — schema ui_emotion_request_v1

Voice Intent (Step 1.7)

kilo/cmd/intent — schema cmd_intent_v1

Request-only (untrusted). Event-driven voice/UI intent envelope.
Includes: STOP, UNLOCK_REQUEST, SET_MODE, ROAM_START/STOP, MAPPING_START/STOP, STATUS.
See "INTENT & HEARTBEAT CADENCE" for publication rules. Safety Gate remains the sole authority.

Robot → Phone (Truth)
Health

kilo/health — schema health_v1

Control Truth

kilo/state/control — schema state_control_v1

Includes:

armed

locked + locked_reason
kilo/state/safety_model — schema state_safety_model_v1

Truth-only publishing of speed-aware safety model parameters and outputs. Additive-only; does not create a second arbiter. Safety Gate remains the only motion authority.


stale_cmd

last_cmd_age_ms

applied {steer, throttle}

relay_killed + relay_reason

Neutral throttle definition (LOCKED):

throttle = 0.0 exactly

Safety Gate Truth

kilo/state/safety — schema state_safety_v1

Reason codes:

Stable

Additive-only

Meaning never changes

Additive reason codes (Phase 2):

- ROLLOVER
- IMPACT

Additive fields (Phase 2):

- imu_ok: bool (true if recent IMU received within imu_ttl_ms)
- imu_age_ms: integer|null (age since last IMU receipt; computed by robot clock)
- imu_tilt_deg: number|null (tilt angle from upright, degrees)
- rollover_latched: bool (true if rollover detected, with hysteresis)
- imu_accel_g: number|null (acceleration magnitude in g units, if provided)
- impact_latched: bool (true if impact detected, with hysteresis)

Alerts

kilo/alerts — schema alert_v1

System State

kilo/state/system — schema state_system_v1

RESERVED PHASE 3–7 CONTRACTS (OPTIONAL UNTIL IMPLEMENTED)
Phase 3 — Perception Summaries

kilo/state/perception_json — schema state_perception_v1

Validity rule (LOCKED):

Invalid or stale perception → Safety Gate MUST DENY
unless a documented fallback exists

Phase 4 — Safety Model

kilo/state/safety_model

Schema (implemented; additive-only):

- schema_version: "state_safety_model_v1"
- ts_ms: integer (robot time)
- profile: "crawl" | "normal" | "sport"
- params:
  - reaction_time_s: number
  - brake_decel_mps2: number
  - buffer_m: number
- inputs:
  - speed_mps: number
- outputs:
  - stop_distance_m: number
  - max_safe_speed_mps: number|null (optional)
  - model_valid: bool
  - model_reason: string ("OK" | "INVALID_INPUT" | "PARAM_MISSING")

Rules (contract-only):

- This topic is **truth only**; consumers must not infer safety model state from cmd/phone topics.
- Computation must be deterministic and logged in tests.
- Additive-only changes: new fields allowed; no renames/removals.

Phase 5 — Mapping

kilo/state/mapping

Phase 6 — Navigation

kilo/state/navigation

Phase 7 — Docking

kilo/state/docking

SAFETY + PERFORMANCE INVARIANTS (INTERFACE-LEVEL)

Loss of drive → neutral (cmd TTL)

Loss of heartbeat → lockout (hb TTL)

Safety Gate deny → no motion

Lock never clears silently

Telemetry bounded and rate-limited

No video over MQTT

Absence of phone must not break robot

FINAL STATUS

SEALED — CANONICAL — NO REWRITE REQUIRED

Any change that violates this document is:

OUT OF SCOPE

REQUIRES CHANGE TICKET

BLOCKS PHASE ADVANCEMENT

If you want, next step is to:

freeze this as /docs/INTERFACE_CONTRACTS.md

tag it in the Decisions Ledger

and define Phase 1 acceptance checks that explicitly reference this contract

This is now a solid rail system.
