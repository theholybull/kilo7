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

SCHEMA EVOLUTION (DRIFT KILLER)

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

Mode Request

kilo/ui/mode_request — schema ui_mode_request_v1

Emotion Request

kilo/ui/emotion_request — schema ui_emotion_request_v1

Robot → Phone (Truth)
Health

kilo/health — schema health_v1

Control Truth

kilo/state/control — schema state_control_v1

Includes:

armed

locked + locked_reason

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

Alerts

kilo/alerts — schema alert_v1

System State

kilo/state/system — schema state_system_v1

RESERVED PHASE 3–7 CONTRACTS (OPTIONAL UNTIL IMPLEMENTED)
Phase 3 — Perception Summaries

kilo/state/perception — schema state_perception_v1

Validity rule (LOCKED):

Invalid or stale perception → Safety Gate MUST DENY
unless a documented fallback exists

Phase 4 — Safety Model

kilo/state/safety_model

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