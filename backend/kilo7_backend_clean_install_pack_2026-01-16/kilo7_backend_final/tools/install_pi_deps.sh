#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update

# Common deps
sudo apt-get install -y \
  git curl jq htop \
  python3 python3-venv python3-pip \
  python3-yaml python3-paho-mqtt \
  mosquitto mosquitto-clients \
  udev usbutils

# pigpio / pigpiod strategy:
# - Debian/Raspberry Pi OS: 'pigpio' package provides pigpiod + systemd unit.
# - Ubuntu Jammy on Raspberry Pi: 'pigpio' package often missing; we install client libs/tools
#   and (if needed) build + install pigpiod from source.

have_pigpio_pkg=0
if apt-cache policy pigpio 2>/dev/null | grep -q 'Candidate:'; then
  if ! apt-cache policy pigpio 2>/dev/null | awk '/Candidate:/{print $2}' | grep -qx '(none)'; then
    have_pigpio_pkg=1
  fi
fi

if [ "$have_pigpio_pkg" = "1" ]; then
  sudo apt-get install -y pigpio python3-pigpio
else
  sudo apt-get install -y python3-pigpio pigpio-tools libpigpiod-if2-1 || true

  if ! command -v pigpiod >/dev/null 2>&1; then
    echo "pigpiod not found via apt; building from source..."
    tmpdir=$(mktemp -d)
    trap 'rm -rf "$tmpdir"' EXIT
    git clone --depth 1 https://github.com/joan2937/pigpio "$tmpdir/pigpio"
    make -C "$tmpdir/pigpio" -j"$(nproc)"
    sudo make -C "$tmpdir/pigpio" install
    sudo ldconfig
  fi

  # Install a minimal pigpiod systemd unit if the distro didn't provide one.
  if ! systemctl list-unit-files | grep -q '^pigpiod\.service'; then
    echo "Installing pigpiod systemd unit (/etc/systemd/system/pigpiod.service)"
    sudo tee /etc/systemd/system/pigpiod.service >/dev/null <<'UNIT'
[Unit]
Description=pigpio daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/pigpiod -l
Restart=on-failure
RestartSec=1

[Install]
WantedBy=multi-user.target
UNIT
    sudo systemctl daemon-reload
  fi
fi

# Ensure pigpio daemon is enabled for pwm.mode=pigpio
sudo systemctl enable --now pigpiod || true

echo "Deps installed."
if command -v pigpiod >/dev/null 2>&1; then
  echo "pigpiod: $(command -v pigpiod)"
else
  echo "WARNING: pigpiod still not found. PWM mode 'pigpio' will not be available."
fi

echo "ROS 2 install is handled separately per Phase 1.1 rules."
