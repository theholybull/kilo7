# KILO .7 — Project Path & Guardrails

## Non‑negotiables
1. Offline first (no cloud dependency for core behaviors).
2. Headless robot (phone = sensors + UI only).
3. One authority for motion (Safety Gate is the only stop arbiter).
4. Stability beats features (core control stays stable even if perception/nav fails).

## Build order (mandatory)
1) Bring‑Up (PWM, dead‑stop, relay kill, telemetry)
2) Safety Gate (rollover, impact, component loss)
3) Perception for avoidance (front depth, rear ToF)
4) Speed‑aware safety model (stop distance scales with speed)
5) Mapping / localization (2D)
6) Navigation (recoveries)
7) Docking/charging (last)

### Step 1.8 — UI Truth + Soak (proposal)
- Scope: finalize voice‑only interface semantics on the phone; ensure UI lock/emotion derives only from `/kilo/state/safety_json` and `/kilo/state/control_json` (no request-topic inference).
- Tests: soak IMU + intent streams; add invariants for intent handling and UI truth mapping; verify no raw drive paths are exposed.
- Guardrails: offline‑first, single motion authority, relay hard‑kill remains in series; no autonomy behaviors enabled by default.

## Zones
**Green (always allowed):** stability fixes, safety, observability, field tuning of existing params, reversible refactors that reduce failure modes.  
**Yellow (requires a ticket):** new sensors, new autonomy behaviors, comms changes, control loop changes, safety trigger changes, enabling heavy pipelines by default.  
**Red (forbidden without explicit approval):** cloud dependencies, always‑on mapping, multiple stop authorities, hidden background loops/services, changes that touch safety/control “temporarily”.

## Experiment rule
Experiments must be isolated, feature‑flagged, or run on recorded data. Baseline must remain recoverable.

## Progress definition
Progress = longer runtime, fewer knobs, clearer failure reasons, easier recovery. Not “more features.”
