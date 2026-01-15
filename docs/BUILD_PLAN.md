0) Global rules (apply to every step)

Robot is headless. Phone = sensors + UI only.

One Safety Gate is the only stop authority.

Core control must run without perception/nav.

Every step produces:

Acceptance checks (commands + expected outputs)

Update docs/PROJECT_STATE.md with what changed and what’s next

Phase 1 — Bring-up (baseline must become boring)
Step 1.1 — Dependency + tooling install (Pi)

Goal: Pi has everything needed for ROS 2 dev/runtime, MQTT, camera support tooling, and build utilities.

Deliverable: “Dependency manifest” recorded in repo and install script committed.

Install targets (decision-aligned)

ROS 2 (Debian packages)

Build tools: cmake, colcon, vcs, python venv tooling

MQTT broker + client tools: mosquitto, mosquitto-clients

Git, curl, jq (optional), htop

Udev utilities, USB tools

(DepthAI comes later when we wire OAK-D; don’t enable heavy pipelines yet)

Acceptance gate

ros2 --help works

colcon --help works

mosquitto -h works

mosquitto_sub -h works

Reboot and confirm nothing is wedged

If gate fails: fix installs only—no code changes.

Step 1.2 — Repo checkout + workspace skeleton (Pi)

Goal: A ROS 2 workspace exists and builds cleanly with an empty “hello” package.

Deliverable: robot/ contains ros_ws/ skeleton and a minimal package.

Acceptance gate

colcon build succeeds

source install/setup.bash succeeds

ros2 pkg list | grep <package> shows it

Step 1.3 — PWM driver node (Ackermann) with dead-stop

Goal: A ROS 2 node outputs PWM at 60 Hz to steering + throttle and goes neutral on stale command.

Constraints

No perception, no nav

No background heavy loops outside the 60 Hz control loop

Stale timeout triggers neutral + lock

Acceptance gate

Node starts and stays running

Command updates steering/throttle

Stop command neutralizes throttle

If commands stop → neutral within timeout

Step 1.4 — Hardware kill relay (GPIO)

Goal: GPIO-controlled relay kill works independent of ROS node state.

Acceptance gate

Relay can be toggled reliably

Kill cuts power to drive system as designed

On boot, relay defaults to safe state

Step 1.5 — Status/telemetry baseline (robot publishes state)

Goal: Robot publishes health + control status so UI can show “what’s happening.”

Acceptance gate

MQTT topics publish:

robot health

control state (armed, locked, stale)

last command timestamp

Rate-limited (no spam)

Phase 2 — Safety Gate (single stop authority)
Step 2.1 — Safety Gate node (no sensors yet; stubs)

Goal: A Safety Gate exists as a single ROS node that other modules must obey.

Acceptance gate

Gate outputs safe_to_move + reason

When gate = false, control node refuses motion

Lockout behavior is explicit and recoverable

Step 2.2 — Phone IMU ingest (MQTT → ROS) + rollover rule

Goal: Phone IMU arrives and rollover triggers safety stop.

Acceptance gate

IMU messages arrive reliably over MQTT

Rollover condition flips Safety Gate false with reason “rollover”

Recovery method works (manual clear / safe reset)

Step 2.3 — Impact detection (accel + jerk + context)

Goal: Impact triggers stop without false positives when idle.

Acceptance gate

Impact event flips Safety Gate false with reason “impact”

Context gate prevents triggers while stationary unless commanded motion exists

Step 2.4 — Component loss detection

Goal: If critical component data goes stale, Safety Gate stops.

Acceptance gate

Stale timers for IMU/camera/tof publish “component_loss”

Gate denies movement until restored or explicitly overridden (if allowed)

Phase 3 — Perception for avoidance (keep it minimal)
Step 3.1 — OAK-D bring-up (RGB only first)

Goal: OAK-D RGB stream is accessible locally without destabilizing control.

Acceptance gate

RGB pipeline runs at chosen FPS without pegging CPU

Control loop remains stable while camera runs

Step 3.2 — OAK-D depth obstacle distance (front)

Goal: Produce a single front obstacle distance feed for Safety Gate / assist mode.

Acceptance gate

Depth produces stable nearest-distance estimate

Gate uses it to stop when too close

False stops are manageable (logged, explainable)

Step 3.3 — Rear ToF + rear cam ingest

Goal: Rear ToF supports reverse safety and recovery backing.

Acceptance gate

Rear ToF data is stable

Reverse motion is vetoed when rear too close

Rear cam available for UI/debug

Phase 4 — Speed-aware safety model (this prevents Viam pain)
Step 4.1 — Speed profiles + dynamic stop distance

Goal: Stop distance scales with speed; tuning is profile-based.

Acceptance gate

Profiles exist (crawl/normal/sport)

Computed stop distance published and visible

Increasing speed increases stop distance automatically

Step 4.2 — Field tuning workflow

Goal: Tuning can be done in the field without breaking everything.

Acceptance gate

Parameters adjustable at runtime (RAM apply)

Changes are logged + reversible

“Commit” requires explicit action

Phase 5 — Mapping & Localization (2D, opt-in)
Step 5.1 — Mapping mode (explicit)

Goal: Build a map in mapping mode only.

Acceptance gate

Mapping can be started/stopped explicitly

Control remains responsive

Map saved to USB with a name

Step 5.2 — Localization mode (default)

Goal: Load a named map from USB and localize against it.

Acceptance gate

Select map name

Localization stable enough for navigation experiments

If localization fails, system degrades safely (no wedging)

Phase 6 — Navigation (Nav2-style behavior)
Step 6.1 — Simple autonomy behaviors (no docking)

Goal: Go-to-goal with recovery behaviors.

Acceptance gate

Waypoint navigation works in a controlled space

Recovery behaviors don’t fight Safety Gate

Manual override always works

Phase 7 — Docking/Charging (last)
Step 7.1 — AprilTag-based approach

Goal: Approach dock reliably.

Acceptance gate

Tag detection stable near dock

Approach routine consistent and safe

Step 7.2 — Pull-through docking + confirmation

Goal: Dock and confirm contact/charging.

Acceptance gate

Switch confirms contact

Charge sensing confirms charging

Failures back off and retry safely
