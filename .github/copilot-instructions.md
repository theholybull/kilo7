# KILO .7 — AI Coding Agent Instructions

**Last Updated:** 2026-01-22 | **Project Phase:** Phase 1 Bring-up (Steps 1.4–1.6 complete, Step 1.7 in progress)

## Quick Orientation

**KILO .7** is an offline-first autonomous rover (ROS 2 + Ackermann drive). The codebase lives on a removable USB drive and must prioritize **stability over features**. 

### Non-Negotiables (Read These First)
1. **Offline first** — no cloud dependencies for core behaviors
2. **Single motion authority** — `Safety Gate` (ROS node) alone decides if motion is allowed
3. **Headless robot** — phone = sensors + UI only; robot runs autonomously on Raspberry Pi
4. **Core control is isolated** — PWM + dead-stop + relay kill must work even if perception/nav crashes
5. **Voice-only user interface** (Step 1.7) — phone sends only intents + IMU, never raw drive commands

---

## Architecture Essentials

### Safety Authority Model (Immutable)

**`/kilo/state/safety_json`** (published by `kilo_safety_gate`)
- **Only authoritative source** for `safe_to_move` (bool)
- Schema: `state_safety_v1` (exact match required)
- Fuses: rollover, impact, component loss, heartbeat staleness
- Gate emits `safe_to_move: false` → all PWM output must be exactly `0.0` (hard requirement)

**`/kilo/state/control_json`** (published by `kilo_control_pwm`)
- Reflects **what is actually applied** to motors
- Schema: `state_control_v1`
- Must cache gate decision + relay state; enforces `throttle = 0.0` when gate denies
- Caches gate response time for observability

### Hardware Fail-Safes

- **GPIO17 relay kill** (ESC power) — fail-closed on boot (HIGH=KILL, LOW=RUN)
- **PWM at 60Hz on Pi** — immediate dead-stop on command loss (no signal = throttle→0)
- **No cloud dependency** for any safety decision

### Three-Layer Motion Control
1. **ROS layer:** `Safety Gate` logic + `Control PWM` enforcement
2. **Hardware layer:** GPIO relay (fail-closed, cannot be defeated by software)
3. **MQTT layer:** Command TTL + schema validation (mismatched schema → dropped, no TTL refresh)

---

## Code Locations & Key Nodes

### ROS Packages
- **`robot/ros_ws/src/kilo_core/`** — Main control backend
  - `safety_gate.py` — Runs logic-only; emits truth on `/kilo/state/safety_json`
  - `control_pwm.py` — Reads gate decision, applies to motors, caches state on `/kilo/state/control_json`
  - `relay_kill.py` — GPIO17 relay manager; publishes status on `/kilo/hw/relay_status_json`
  - `mqtt_bridge.py` — MQTT→ROS bridge; enforces schema validation on all cmd topics
  - `util.py` — Shared schema/config helpers
  - `config/` — YAML configs for all nodes (relay mode, PWM channels, etc.)

### Documentation Spine (Single Source of Truth)
- `docs/PROJECT_STATE.md` — **Read this first.** Current phase/step, what's working, test commands
- `docs/PROJECT_CHARTER.md` — Purpose, scope, architecture decisions
- `docs/PROJECT_PATH.md` — Build order, guardrails (Green/Yellow/Red zones)
- `docs/AUTHORITY_CHAIN.md` — Truth topics, schema contracts, immutable authority model
- `docs/DECISIONS_LEDGER.md` — Append-only design decisions (ROS 2, Ackermann, MQTT, etc.)
- `docs/BUILD_INSTRUCTIONS.md` — Chat header format for developer sessions

---

## Developer Workflows

### Understanding Current State
1. Read `docs/PROJECT_STATE.md` **first** — it's the source of truth
2. Check `docs/AUTHORITY_CHAIN.md` for topic contracts
3. Look at test commands in PROJECT_STATE (e.g., `test_step_1_6_invariants.py`)

### Building & Testing
```bash
# Source ROS + workspace
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash

# Build
cd /opt/kilo7/robot/ros_ws
colcon build --packages-select kilo_core

# Run integration tests (current phase)
python3 /opt/kilo7/robot/test_step_1_6_invariants.py

# Verify single-publisher invariant (Safety Gate)
ros2 topic info /kilo/state/safety_json --verbose
```

### Session Protocol (Before Any Work)
1. Paste header from `docs/BUILD_HEADERS.md` (STANDARD/EXPERIMENT/HOTFIX)
2. Paste `docs/PROJECT_STATE.md` into the chat header
3. Label intent: EXPLAIN ONLY | RISK CHECK | REVIEW | CHANGE REQUEST | **IMPLEMENT**
4. Work only within CURRENT GOAL (scope is law)
5. **End session by updating:**
   - `docs/PROJECT_STATE.md` (new status)
   - `docs/CHANGELOG.md` (if code changed)
   - `docs/LESSONS_LEARNED.md` (if applicable)

---

## Critical Patterns & Conventions

### Topic Naming & Authority
- **`/kilo/cmd/*`** — untrusted requests (may be denied by gate)
  - `cmd_drive_v1`, `cmd_stop_v1`, `cmd_heartbeat_v1`, `cmd_unlock_v1` (all schema version required)
  - `cmd_intent_v1` (Step 1.7 — voice-only interface; never raw drive)
- **`/kilo/state/*`** — authoritative truth (single publisher per topic)
  - `state_safety_json`, `state_control_json`
- **`/kilo/hw/*`** — hardware status (relay, GPIO state)
- **`/kilo/phone/*`** — untrusted phone sensor inputs (IMU, GPS)
  - `imu_json` (Step 1.7 — phone_imu_v1 schema required)

### Schema Validation Rules
- Every MQTT message must include `ts_ms` (integer, milliseconds since epoch)
- Wrong `schema_version` on cmd topics → **dropped immediately, TTL not refreshed** (critical safety property)
- Valid cmd = correct schema + ts_ms + steer/throttle in range
- Consumers must read robot's computed staleness, not phone's clock

### Speed-Aware Safety Model
- Stop distance scales with velocity (buffer + reaction time + braking)
- Use **profiles** (crawl/normal/sport) rather than fixed thresholds
- One authority (Safety Gate) owns all stop decisions; prevents "panic stop forever"

### Safety Gate Decision Priority (Locked)
1. **LOSS_OF_COMMAND** (heartbeat missing) — overrides other faults, fail-closed
2. **EXPLICIT_STOP** (latched) — highest priority once set
3. **COMPONENT_MISSING** (sensor/relay fault)
4. **ROLLOVER, IMPACT, etc.**
5. **OK** (all systems nominal)

---

## Building in Green Zone (Always Allowed)

- Stability fixes, observability improvements
- Field tuning of existing parameters
- Safety improvements
- Reversible refactors that reduce failure modes
- Test coverage

## Yellow Zone (Requires Ticket & Justification)

- New sensors, new autonomy behaviors
- Comms topology changes
- Control loop frequency/timing changes
- Safety trigger adjustments
- Enabling heavy pipelines (mapping, vision) by default

## Red Zone (Forbidden Without Explicit Approval)

- Cloud dependencies
- Multiple stop authorities
- Hidden background loops/services
- Changes to safety/control "temporarily"

---

## Testing & Invariants

**Current Verification (Step 1.6):** 6 integration test invariants in `robot/test_step_1_6_invariants.py`

1. ✅ Single publisher on `/kilo/state/safety_json` (Safety Gate only)
2. ✅ Single publisher on `/kilo/state/control_json` (Control PWM only)
3. ✅ Gate `safe_to_move=false` → Control applies exactly `throttle=0.0`
4. ✅ Schema mismatch on cmd topics → dropped, no TTL refresh
5. ✅ Relay state reflected in Control caches
6. ✅ Legacy HTTP gate masked (`:8098` not listening)

When extending, **add new invariants to the test file**, do not remove old ones.

---

## Step 1.7 — Voice Intent + IMU Integration (In Progress)

### Voice Intent Contract (Locked)

**MQTT Topic:** `kilo/cmd/intent`  
**ROS Topic:** `/kilo/cmd/intent_json`  
**Schema:** `cmd_intent_v1` (exact match required)

**Required Fields:**
- `schema_version: "cmd_intent_v1"` (exact)
- `ts_ms: integer` (milliseconds since epoch)
- `intent: string` (enum: STOP, UNLOCK_REQUEST, SET_MODE, ROAM_START, ROAM_STOP, MAPPING_START, MAPPING_STOP, STATUS)

**Optional Fields:**
- `args: object` (e.g., `{"mode": "IDLE" | "ROAM" | "MAP"}` for SET_MODE)
- `utterance_id: string` (for correlation)
- `confidence: float` (optional confidence score)

**Rules:**
- Phone publishes intent requests **only** — never raw `steer/throttle`
- Intent is a request, not a command — Safety Gate alone decides action
- STOP intent → Safety Gate denies + applied throttle forced 0.0

### Phone IMU Contract (Locked)

**MQTT Topic:** `kilo/phone/imu`  
**ROS Topic:** `/kilo/phone/imu_json`  
**Schema:** `phone_imu_v1` (exact match required)

**Required Fields:**
- `schema_version: "phone_imu_v1"` (exact)
- `ts_ms: integer` (milliseconds since epoch)
- **At least one orientation representation:**
  - Roll/pitch/yaw (floats), OR
  - Quaternion (`{"x": float, "y": float, "z": float, "w": float}`), OR
  - Accel/gyro (impl-dependent)

**Rules:**
- Phone IMU is **untrusted** input
- Missing required fields → bridge drops + alerts
- Robot uses IMU for rollover detection + stability (Phase 2+)

### UI Emotion Rule (Locked — Step 1.7)

UI emotion/lockout indicators are derived **ONLY** from authoritative Pi truth topics:

**Sources:**
- `kilo/state/safety`: `safe_to_move`, `reason`, `latched`, `override_required`
- `kilo/state/control`: `locked`, `locked_reason`, `relay_killed`, `relay_reason`, `applied.throttle`
- `kilo/state/system`: battery status

**NEVER use for UI emotion:**
- MQTT connection status
- "Command sent" / "Intent received"
- Request topic activity

This ensures UI cannot misrepresent motion authority or applied state.

---

## Common Tasks

### Adding a New Node
1. Create in `robot/ros_ws/src/kilo_core/kilo_core/my_node.py`
2. Add to `setup.py` entry_points under `console_scripts`
3. Create systemd service in `robot/ros_ws/run/systemd/` if long-running
4. Document truth topics in `docs/AUTHORITY_CHAIN.md` (additive only)
5. Add invariant test to `test_step_1_6_invariants.py`

### Changing a Truth Topic Schema
- **Additive only** — never rename or remove fields
- Bump `schema_version: "state_*_v2"` and use new version everywhere
- Old consumers must handle both `v1` and `v2` (test both)
- Update `docs/INTERFACE_CONTRACT.md` (additive only)
- Update PROJECT_STATE with migration notes

### Field Tuning Without Code Changes
- Edit `robot/ros_ws/src/kilo_core/config/*.yaml`
- Restart node: `systemctl restart kilo7-<node-name>`
- No rebuild needed (YAML is hot-loadable)

---

## References & Exit Criteria

- **Charter:** `docs/PROJECT_CHARTER.md` (purpose, scope, milestones)
- **Contracts:** `docs/INTERFACE_CONTRACT.md` (MQTT topic schemas, immutable)
- **Authority Model:** `docs/AUTHORITY_CHAIN.md` (truth topics, single publisher rule)
- **Build Order:** `docs/PROJECT_PATH.md` (guardrails, zones, non-negotiables)
- **Current Status:** `docs/PROJECT_STATE.md` (what works, test commands, next step)

---

## Quick Checklist Before Pushing

- [ ] No new cloud dependencies
- [ ] No additional motion authorities created
- [ ] Truth topics remain single-publisher (verify with `ros2 topic info`)
- [ ] Schema changes are additive-only (if any)
- [ ] All existing tests still pass
- [ ] Updated `docs/PROJECT_STATE.md` with new status
- [ ] Updated `docs/CHANGELOG.md` with what changed
- [ ] Verified on hardware (not just code review)
