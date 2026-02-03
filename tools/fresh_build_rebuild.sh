#!/usr/bin/env bash
set -euo pipefail

# KILO.7 fresh install rebuild (Ubuntu 22.04, ROS 2 Humble)
# Safe, idempotent steps for future rebuilds.

REPO_HOME="/home/kilo/kilo7"
RUNTIME_HOME="/opt/kilo7"
ROS_SETUP="/opt/ros/humble/setup.bash"

log() { echo "[rebuild] $*"; }

require_root() {
  if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    log "Please run as root: sudo $0"
    exit 1
  fi
}

setup_hostname() {
  local hostname="kilo-dev"
  if [ "$(hostname)" != "$hostname" ]; then
    log "Setting hostname to $hostname"
    hostnamectl set-hostname "$hostname"
  else
    log "Hostname already set: $hostname"
  fi
}

install_base_packages() {
  log "Installing base packages"
  apt-get update -y
  apt-get install -y \
    avahi-daemon \
    mosquitto mosquitto-clients \
    python3-pip \
    python3-rpi.gpio \
    rsync
}

install_python_deps() {
  log "Installing python deps"
  python3 -m pip install --upgrade pip
  python3 -m pip install pigpio pyyaml paho-mqtt
}

configure_mosquitto() {
  log "Configuring mosquitto to listen on all interfaces"
  if ! grep -q "listener 1883 0.0.0.0" /etc/mosquitto/mosquitto.conf; then
    cat >> /etc/mosquitto/mosquitto.conf << 'EOF'

# KILO.7: Listen on all interfaces for robot and phone connectivity
listener 1883 0.0.0.0
allow_anonymous true
EOF
  fi
  systemctl enable mosquitto
  systemctl restart mosquitto
}

verify_mdns() {
  log "Verifying mDNS (kilo-dev.local)"
  systemctl enable avahi-daemon
  systemctl restart avahi-daemon
  ping -c 1 kilo-dev.local >/dev/null 2>&1 || true
}

sync_repo_to_runtime() {
  log "Syncing repo to runtime location"
  mkdir -p "$RUNTIME_HOME"
  rsync -av "$REPO_HOME/" "$RUNTIME_HOME" \
    --exclude='.git' \
    --exclude='robot/ros_ws/build' \
    --exclude='robot/ros_ws/install' \
    --exclude='robot/ros_ws/log'
}

build_kilo_core() {
  log "Building kilo_core"
  if [ ! -f "$ROS_SETUP" ]; then
    log "ROS setup not found: $ROS_SETUP"
    log "Install ROS 2 Humble before building."
    exit 1
  fi
  source "$ROS_SETUP"
  cd "$RUNTIME_HOME/robot/ros_ws"
  colcon build --packages-select kilo_core
}

post_checks() {
  log "Post checks"
  systemctl is-active mosquitto || true
  ss -tln | grep 1883 || true
  ls -la "$RUNTIME_HOME/robot/ros_ws/src/kilo_core/config/kilo.yaml" || true
}

main() {
  require_root
  setup_hostname
  install_base_packages
  install_python_deps
  configure_mosquitto
  verify_mdns
  sync_repo_to_runtime
  build_kilo_core
  post_checks
  log "Done. Review docs/PROJECT_STATE.md for next concrete step."
}

main "$@"
