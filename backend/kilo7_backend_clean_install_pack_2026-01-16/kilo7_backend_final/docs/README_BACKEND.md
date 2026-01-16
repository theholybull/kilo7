# KILO7 Backend (Phase 1–2) — Contract-Correct Build

This package implements the Phase 1–2 backend nodes for KILO .7:

- MQTT bridge (untrusted cmd/ui/phone -> ROS JSON; ROS truth -> MQTT truth)
- Safety Gate stub (single stop authority, publishes `kilo/state/safety`)
- Control PWM (pigpio 60Hz, TTL + lock semantics, publishes `kilo/state/control`)
- Relay kill stub (fail-loud, publishes relay truth via ROS and into control truth)

## Deployment assumptions (REQUIRED)

This package does not create system users or `/opt` layout automatically.

You MUST provide:
- A Linux user named: `kilo`
- Install root: `/opt/kilo7` (owned by `kilo`)
- ROS workspace at: `/opt/kilo7/robot/ros_ws`

Systemd units are written against those assumptions unless overridden via:
- `/etc/default/kilo7`
- `/opt/kilo7/kilo7-backend.env`

## PWM mode

This build is configured for `pwm.mode=pigpio`.

REQUIRED MINIMUM continuity for pigpio:
- install `pigpio` + `python3-pigpio`
- enable `pigpiod.service` at boot
- `kilo7-control.service` orders After/Wants `pigpiod.service`

## Install deps (Pi)

```bash
sudo ./tools/install_pi_deps.sh
```

## Build workspace

From `/opt/kilo7`:

```bash
source /opt/ros/humble/setup.bash
cd /opt/kilo7/robot/ros_ws
colcon build --merge-install --symlink-install --event-handlers console_direct+
source install/setup.bash
```

Notes:
- This workspace is intended to be a ROS overlay; `install/setup.bash` must extend `AMENT_PREFIX_PATH` to include the workspace prefix.
- If `ros2 pkg list` does not show `kilo_core`, re-check `src/kilo_core/hooks/ament_prefix_path.dsv` (must *not* contain `{prefix}`).

## Run nodes (manual)

```bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
ros2 run kilo_core mqtt_bridge
ros2 run kilo_core safety_gate
ros2 run kilo_core relay_kill
ros2 run kilo_core control_pwm
```

## Systemd

Systemd unit files are located in:
`robot/ros_ws/src/kilo_core/systemd/`

Copy them into `/etc/systemd/system/` and enable as needed.
