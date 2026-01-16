BUILD_PLAN.md — KILO .7 (Revised to match ROS2+MQTT backend package)

Version: v1.1-revA
Last updated: 2026-01-16
Status: Canonical build plan (execution order + gates)
Contract: INTERFACE_CONTRACTS v1.2 (sealed, additive-only)

CHANGE NOTE (2026-01-16): This plan is revised to reflect that the “backend” is a ROS 2 workspace package and therefore ROS 2 is a hard prerequisite before installing/building the backend bundle.
CHANGE NOTE (2026-01-16): PROJECT_STATE.md is immutable. Build progress is recorded in BUILD_LOG.md / CHANGELOG.md instead.
CHANGE NOTE (2026-01-16): Transport is MQTT only; no HTTP assumptions are allowed.

0) Global rules (apply to every step)

Robot is headless. Phone = sensors + UI only.

One Safety Gate is the only stop authority.

Core control must run without perception/nav.

Offline-first.

Additive-only interfaces: never rename/remove fields/topics.

Every step produces:

Acceptance checks (commands + expected outputs)

A short entry appended to docs/BUILD_LOG.md (or CHANGELOG.md) describing:

what changed

what was verified

known issues / next step

CHANGE NOTE: Replaces “Update docs/PROJECT_STATE.md…” because PROJECT_STATE is locked.

PHASE 0 — Prereq Gate (MUST PASS before any backend install)
Step 0.1 — ROS 2 baseline must exist

Goal: The Pi can build and run ROS 2 Python nodes using colcon.

REQUIRED (gate):

source /opt/ros/<distro>/setup.bash succeeds

ros2 --help works

python3 -c "import rclpy; print('rclpy ok')" works

colcon --help works

Acceptance checks (copy/paste):

source /opt/ros/*/setup.bash
ros2 --help >/dev/null
python3 -c "import rclpy; print('rclpy ok')"
colcon --help >/dev/null


If gate fails: fix ROS 2 install only. No backend debugging.

PHASE 1 — Bring-up (baseline must become boring)
Step 1.1 — Dependency + tooling install (Pi)

Goal: Pi has system tools needed for the backend bundle (MQTT + build tooling + hardware daemons).

Required targets:

Mosquitto broker + CLI: mosquitto, mosquitto-clients

Utilities: git, curl, jq (optional), htop, unzip

Hardware daemon deps if using real PWM: pigpio + pigpiod

Python deps used by nodes/scripts (as defined by backend package scripts)

Acceptance checks:

mosquitto -h >/dev/null
mosquitto_sub -h >/dev/null
unzip -v >/dev/null

Step 1.2 — Install layout + runtime user (deployment assumptions)

Goal: Deploy layout matches backend README and systemd assumptions.

Required (per backend README):

Linux user: kilo

Install root: /opt/kilo7

Workspace path: /opt/kilo7/robot/ros_ws

Acceptance checks:

id kilo
test -d /opt/kilo7
test -d /opt/kilo7/robot/ros_ws


If you’re installing on a dev machine and want a different path, you must provide overrides via:

/etc/default/kilo7 and/or

/opt/kilo7/kilo7-backend.env

Step 1.3 — Backend package install (zip drop) + dependency script

Goal: The backend bundle is unpacked and deps are installed, without “winging it.”

Acceptance checks:

ls -la /opt/kilo7
test -x /opt/kilo7/tools/install_pi_deps.sh


Notes:

This step assumes ROS 2 already exists (Phase 0 passed).

This step must not start services yet unless explicitly required by the package.

Step 1.4 — Build the ROS workspace (colcon)

Goal: Workspace builds cleanly and produces install artifacts.

Acceptance checks:

sudo -u kilo bash -lc '
source /opt/ros/*/setup.bash
cd /opt/kilo7/robot/ros_ws
colcon build
test -f install/setup.bash
'

Step 1.5 — MQTT broker enablement + retained truth topics exist

Goal: MQTT broker runs and backend publishes retained truth topics.

Acceptance checks:

systemctl is-active --quiet mosquitto && echo mosquitto_ok
mosquitto_sub -t kilo/health -C 1 -v
mosquitto_sub -t kilo/state/safety -C 1 -v
mosquitto_sub -t kilo/state/control -C 1 -v


Expected:

Each payload is JSON

Each includes schema_version and ts_ms

kilo/state/safety should reflect “latched/override_required” defaults (per backend README)

kilo/state/control must show lock + stale behavior truthfully

Step 1.6 — Control + stop semantics are enforceable (no motion without truth)

Goal: Confirm dead-stop, stale neutral, and lock semantics are functioning.

Acceptance checks (minimum):

Stop request causes lock + applied throttle neutral

Stale cmd causes neutral within configured TTL

Heartbeat stale causes lockout (if enabled/configured)

(Exact commands depend on how you inject test messages—MQTT publish or ROS internal topic—but outcomes are non-negotiable.)

PHASE 2 — Safety Gate (single arbiter)
Step 2.1 — Safety Gate is the ONLY motion authority

Goal: Control enforcement obeys Safety Gate deny always.

Acceptance checks:

When kilo/state/safety.safe_to_move=false, kilo/state/control.applied.throttle must be neutral (0.0)

Reason codes are stable and additive-only

Step 2.2 — Phone IMU ingest (MQTT → robot truth)

Goal: Phone IMU can arrive, and Safety Gate can consume it (even if rules are stubbed early).

Acceptance checks:

Messages arrive on kilo/phone/imu

Robot truth indicates validity/staleness via kilo/state/* (not inferred from phone stream)

Step 2.3 — Component loss detection (staleness)

Goal: Stale critical components cause deny/lock behavior as defined.

Acceptance checks:

If IMU (or required sensor) goes stale beyond TTL → Safety Gate deny with a stable reason code

Recovery is explicit and documented; nothing clears silently

PHASE 3–7 — Reserved (do not implement early; only preserve contracts)

Phase 3: Perception summaries only (no video over MQTT)
Phase 4: Speed-aware safety model
Phase 5: Mapping/localization opt-in
Phase 6: Navigation with recoveries
Phase 7: Docking/charging

Guardrails (must not change later):

Requests vs truth separation stays intact (kilo/cmd/* vs kilo/state/*)

Safety Gate reason codes are additive-only

Control node command-timeout semantics remain stable

No perception/nav dependency added into core control loop