KILO .7 — Backend (ROS 2 + MQTT)

Status: Phase 1–2 complete
Scope: Bring-up, Safety Gate, real PWM control
Contract: INTERFACE_CONTRACTS — KILO .7 v1.2 (sealed, additive-only)

This backend is the authoritative robot runtime for KILO .7.

It is:

Offline-first

Headless (phone = sensors + UI only)

ROS 2–based

MQTT-connected

Governed by a single Safety Gate

This is not a demo stack and does not attempt to be forgiving. If a component lies, goes stale, or violates contract, the robot stops.

Architecture (what runs where)

Robot (this repo):

ROS 2 nodes

PWM output (steer + throttle)

Safety Gate (single motion authority)

MQTT bridge (transport only, no logic)

Hardware kill relay

Phone / UI (out of scope here):

Publishes commands + sensors

Subscribes to robot truth

Never trusted for authority

Non-Negotiables (do not violate these)

Safety Gate is the only motion authority

Loss of command → throttle neutral

Loss of heartbeat → lockout

Lock never clears silently

Robot truth is published only on kilo/state/*

Robot never infers truth from phone topics

Interfaces are additive-only

If you need to change any of that, you need a change ticket.

Deployment Assumptions (REQUIRED)

This package does not create users or filesystem layout for you.

You MUST provide:

Linux user: kilo

Install root: /opt/kilo7

ROS workspace: /opt/kilo7/robot/ros_ws

Systemd units are written against those assumptions unless overridden.

Override points:

/etc/default/kilo7

/opt/kilo7/kilo7-backend.env

If those don’t exist, services will not start. That is intentional.

PWM Modes (IMPORTANT)
Default: mock

No physical PWM output

Truth includes:

applied_simulated = true

pwm_output_ok = false

UI must not treat applied values as physical motion

This mode is safe for development.

Real hardware: pigpio

Uses pigpiod + python3-pigpio

Writes servo pulses every control tick

Truth includes:

applied_simulated = false

pwm_output_ok = true

If you enable pigpio:

pigpio must be installed

pigpiod must be running

Control service must start after pigpiod

The backend enforces this. It will not lie.

What’s in this backend
ROS Nodes

mqtt_bridge

MQTT ↔ ROS JSON bridge

Enforces QoS rules

Drops malformed payloads

Publishes retained robot truth

safety_gate

Single motion authority

Phase 2.1 stub

Publishes:

safe_to_move

reason

latched

override_required

control_pwm

60 Hz control loop

Enforces:

command TTL

heartbeat TTL

lock semantics

Safety Gate deny

Outputs real PWM (pigpio) or explicit simulation

relay_kill

Hardware kill relay

Fail-closed on boot

Never silently NOOP

Truth is always observable

Safety & Truth Guarantees
Control truth (kilo/state/control)

last_cmd_age_ms is always an integer

applied reflects actual hardware output or is explicitly marked simulated

locked_reason is always explicit

Relay state is always visible

Safety truth (kilo/state/safety)

Includes:

safe_to_move

reason

latched

override_required

Alerts (kilo/alerts)

Schema-correct

Bounded

Never retained

Install Summary (short version)
# create runtime user + layout
sudo useradd -m -s /bin/bash kilo || true
sudo mkdir -p /opt/kilo7
sudo chown -R kilo:kilo /opt/kilo7

# unzip backend
sudo -u kilo unzip kilo7_backend_final_build.zip -d /opt/kilo7

# install deps (includes pigpio)
cd /opt/kilo7
sudo ./tools/install_pi_deps.sh

# enable daemons
sudo systemctl enable --now pigpiod
sudo systemctl enable --now mosquitto

# build workspace
sudo -u kilo bash -lc '
source /opt/ros/*/setup.bash
cd /opt/kilo7/robot/ros_ws
colcon build
'


See the full install instructions for step-by-step detail.

What “working” looks like

After startup, these must be true:

kilo/health retained and populated

kilo/state/safety shows latched + override_required

kilo/state/control shows:

last_cmd_age_ms as an int

pwm_output_ok consistent with config

applied_simulated correct

Sending STOP always neutralizes throttle

Loss of heartbeat locks the system

If any of those are false, the backend is not healthy.

What this backend does NOT do (yet)

Mapping

Localization

Navigation

Docking

Perception fusion

Cloud anything

Those come later, without breaking this.

Philosophy (why this is strict)

This backend is designed so:

Bugs are visible, not masked

Missing data causes safe failure

UI never has to guess what the robot is doing

Incremental autonomy does not destabilize core control

If something feels “overly strict,” it’s because loose systems rot.

Next Phases (when ready)

Phase 3: perception summaries (front depth + rear ToF)

Phase 4: speed-aware stopping model

Phase 5+: mapping, navigation, docking

Each phase must preserve everything above.


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
