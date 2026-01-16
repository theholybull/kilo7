# KILO .7 Backend — Change Log

## 2026-01-16 — Install/Overlay Fix Pack (no backend logic changes)

### Fixed
- **ROS overlay discovery bug:** `kilo_core` could build but remain undiscoverable because the package hook `hooks/ament_prefix_path.dsv` incorrectly contained a literal `{prefix}` token. This prevented `AMENT_PREFIX_PATH` from being extended during `install/setup.*` sourcing.
- **Ubuntu Jammy pigpio install:** `tools/install_pi_deps.sh` previously attempted to install the `pigpio` package (not available on many Ubuntu-on-Pi images). The script now:
  - installs `python3-pigpio`, `pigpio-tools`, and `libpigpiod-if2-1` when `pigpio` isn’t available,
  - builds and installs `pigpiod` from source only if it is missing,
  - installs a minimal `pigpiod.service` when the distro doesn’t provide one.

### Notes
- Scope: **install wiring only**. No changes to ROS node behavior, MQTT schemas, topics, or safety logic.
