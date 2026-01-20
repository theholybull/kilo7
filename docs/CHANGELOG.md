# CHANGE_LOG — KILO .7 (Backend)

Rules:
- Log by change ticket (CT-YYYY-MM-DD-RT-###).
- Must include scope, impact, files changed, and verification.
- Contract: INTERFACE_CONTRACTS — KILO .7 v1.2 (additive-only unless explicitly approved).

---

## CT-2026-01-20-RT-002 — Enforce install-only runtime + env sanitizer; avoid ros2run fragility

Date: 2026-01-20
Scope: Runtime execution semantics + service launch robustness (no topic/schema changes intended)

Problem:
- Backend services failed under systemd due to `ros2 run` resolving entry points via dist metadata in a fragile environment.
- Workspace built with `--symlink-install` produced `.egg-link` into `build/`, making imports depend on build overlay leakage.

Change:
- Added environment sanitizer script to strip `ros_ws/build` paths from runtime env vars.
- Rebuilt workspace using `colcon build --merge-install` to ensure real installed package (no egg-link).
- Updated launch flow to run modules directly (`python3 -m kilo_core.<module>`) instead of `ros2 run`.

Impact:
- Stabilizes backend imports under systemd.
- Enforces “install overlay only” semantics at runtime.
- Reduces reliance on dist metadata lookup for service startup.
- No ROS topics/schemas/authority semantics changed by this ticket.

Files changed (repo):
- robot/ros_ws/run/kilo7-sanitize-env.sh (new)
- robot/ros_ws/run/kilo7-control.sh (updated to source sanitizer / align runtime)
- kilo7-backend.env (updated as needed to support runtime consistency)
- robot/ros_ws/src/kilo_core/kilo_core/relay_kill.py (fix import/runtime path handling as required)

Verification:
- `python3 -c "import kilo_core.relay_kill as m; print(m.__file__)"`
  returns install site-packages path.
- `systemctl is-active kilo7-relay-kill kilo7-safety-gate kilo7-mqtt-bridge kilo7-control` => active
- `systemctl show <unit> -p ExecStart` confirms `python3 -m ...` usage.

---

## CT-2026-01-20-RT-003 — Stop tracking ROS build/install/log + python artifacts; tighten gitignore

Date: 2026-01-20
Scope: Repo hygiene / reproducibility (no runtime behavior change intended)

Change:
- `.gitignore` updated to permanently exclude:
  - robot/ros_ws/build/
  - robot/ros_ws/install/
  - robot/ros_ws/log/
  - python caches/artifacts (__pycache__, *.pyc, *.pyo, *.pyd, *.egg-info, *.egg-link)
  - common tool caches (.pytest_cache, .mypy_cache, .ruff_cache, .cache)

Impact:
- Prevents accidental commits of machine-local build products.
- Reduces “it works on my machine” drift and review noise.

Files changed:
- .gitignore

Verification:
- `git ls-files | grep -E '^robot/ros_ws/(build|install|log)/'` => none

---

## CT-2026-01-20-RT-004 — Canonical systemd units in repo + apply script

Date: 2026-01-20
Scope: Deployment consistency (no topic/schema changes intended)

Change:
- Added canonical unit files into repo:
  - robot/ros_ws/run/systemd/kilo7-relay-kill.service
  - robot/ros_ws/run/systemd/kilo7-safety-gate.service
  - robot/ros_ws/run/systemd/kilo7-mqtt-bridge.service
  - robot/ros_ws/run/systemd/kilo7-control.service
- Added apply script:
  - robot/ros_ws/run/systemd/apply-kilo7-systemd.sh
- Units:
  - source ROS setup + workspace install setup
  - source sanitizer
  - run `python3 -m kilo_core.<module>`
  - restart=always, restartsec=1

Impact:
- Makes systemd deployment reproducible from git.
- Eliminates host-only unit drift (single source of truth = repo).

Verification:
- Apply script installs units to `/etc/systemd/system`, daemon-reload, enable, restart.
- `systemctl show <units> -p FragmentPath` points to `/etc/systemd/system/<unit>.service`
- `systemctl is-active <units>` => active, `NRestarts=0` after restart.

---
# Changelog

## 2026-01-19 — Repo-on-Robot Baseline (Option A) + Safe Auto-Pull

### Summary
Established the robot Pi as an authoritative git clone host at `/opt/kilo7` (Option A) and enabled safe automatic repo updates via a pull-only systemd timer.

### Changes Made
- `/opt/kilo7` is now a real git clone tracked to `origin/main`
- Added safe auto-pull tool:
  - `tools/kilo7_git_autopull.sh` (ff-only, refuses when working tree is dirty)
- Added systemd pull automation:
  - `kilo7-git-autopull.service` + `kilo7-git-autopull.timer`
  - `SuccessExitStatus=3` so “dirty tree refusal” is non-fatal (prevents log spam)

### Verification
- `git rev-parse HEAD` returns a commit hash
- `git status --porcelain` clean during auto-pull success
- Auto-pull logs show “Already up to date” when no updates exist

### Result
Guardrails and state validation can function again because runtime and docs now derive from a verifiable repo state on the robot.

---

## 2026-01-19 — Safety Authority Unification / One-Path Enforcement

### Summary
Eliminated a hidden dual-safety-authority condition that violated the “one path to safety” rule. A legacy HTTP-based Safety Gate service (`kilo-safety-gate.service`) could exist alongside the ROS-based Safety Gate (`kilo7-safety-gate.service`), creating potential for forked safety truth and undefined behavior.

### Problem
Two independent safety mechanisms could exist simultaneously:
- **ROS Safety Gate** (`kilo7-safety-gate.service`) publishing authoritative safety truth on `/kilo/state/safety_json`
- **Legacy HTTP Safety Gate** (`kilo-safety-gate.service`) exposing `/safety/*` endpoints on TCP 8098

Even when “disabled,” the legacy unit could be restarted, reintroducing a second safety authority and violating the single-authoritative-path requirement.

### Changes Made
- Fully **disabled and masked** the legacy `kilo-safety-gate.service`
- Removed the unit file and override directory
- Replaced the unit with a **symlink to `/dev/null`** to guarantee it cannot be started or re-enabled
- Reloaded systemd to enforce masking

### Verification
- `systemctl is-enabled kilo-safety-gate.service` ? **masked**
- `systemctl status kilo-safety-gate.service` ? **inactive (dead), masked**
- `ss -ltnp | grep :8098` ? **no listeners**

### Result
- Exactly **one** Safety Gate exists at runtime
- All safety truth flows through a single authoritative chain:
  **Command ? Safety Gate (ROS) ? Control ? Actuators ? State**
- Impossible to regress into a dual-safety-authority condition without deliberate manual reversal

### Status
? Closed

---

## 2026-01-19 — Backend Stabilization — MQTT Bridge UTF-8 Crash Fix

### Fixed
- Resolved fatal UTF-8 decode error in `kilo_core/mqtt_bridge.py` caused by a non-UTF8 cp1252 character (0x96, Windows en-dash) in a docstring.
- Rebuilt ROS workspace to propagate corrected UTF-8 source into `build/` and `install/` overlays.
- Fixed `kilo7-mqtt-bridge.service` ExecStart quoting and ROS environment sourcing so `ros2 run` launches reliably under systemd.
- Prevented `kilo7-control` startup failure due to unset optional ROS environment variables (observed under systemd).

### Changed
- `kilo7-mqtt-bridge.service`: replaced fragile inline ExecStart with a shell-based invocation that explicitly sources ROS and workspace environments before execution.
- `run/kilo7-control.sh`: removed `set -u` and defensively initialized `AMENT_TRACE_SETUP_FILES` to avoid abort on unset variables.

### Verified
- MQTT ? ROS bridge remains active and publishes state topics at expected cadence (no dropouts observed during validation window).
- `/kilo/state/control` publishes live (non-retained) messages.
- `/kilo/state/safety_json` and `/kilo/state/control_json` conform to the defined interface contract.
- MQTT command topics observed and bridged as designed (no schema or topic changes introduced).

### Known Issues
- Safety Gate correctly enforces `OVERRIDE_REQUIRED` and `EXPLICIT_STOP`.
- A mismatch was observed between control state and safety state during diagnosis.

#### Impact
- This mismatch can mislead downstream consumers or UI into believing motion is permitted when it is not.
- Violates the invariant that **Safety Gate is the sole motion authority** if consumers treat control state as authoritative.

#### Status
- Diagnosed only.
- No behavioral changes implemented in this change set.

---

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

---

## 2026-01-16 — Backend Build Plan Alignment (ROS 2 Prerequisite Gate)

Type: Process / Build-plan correction  
Scope: Documentation + build sequencing only  
Code impact: None (backend code unchanged)  
Contract impact: None (INTERFACE_CONTRACT v1.2 remains sealed)

### Summary
Aligned the build plan and install flow with the finalized KILO .7 backend package, which is a ROS 2 workspace and therefore requires ROS 2 to be installed before backend installation and build.

This change corrects an implicit assumption in earlier planning that backend installation could precede ROS 2 setup.

### What Changed
- Introduced an explicit Phase 0: ROS 2 Prerequisite Gate in the build plan.
- Clarified that:
  - The backend bundle is not a standalone runtime.
  - `colcon build`, `rclpy`, and ROS environment sourcing are mandatory before backend build/install steps.

### What Did NOT Change
- Backend code, nodes, or systemd units
- Safety Gate semantics
- Control loop behavior (TTL, lock, heartbeat, relay enforcement)
- MQTT topic contracts or schemas
- Deployment layout assumptions (`/opt/kilo7`, `kilo` user)

### Rationale
The backend README and install package already enforce ROS 2 usage (via colcon, rclpy, and ROS launch expectations). The build plan was updated to match reality, reduce install failures, and prevent “winging it” during setup.

### Impact on Future Builds
All future backend installs must pass the ROS 2 gate before proceeding.

### Risk Assessment
Low risk: documentation-only correction  
High value: prevents invalid installs and wasted debug time

### Approval / Notes
This change restores alignment between backend README, install scripts, systemd assumptions, and build execution order.
