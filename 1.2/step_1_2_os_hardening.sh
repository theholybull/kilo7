#!/usr/bin/env bash
set -euo pipefail

# KILO .7 — Step 1.2 OS Hardening (Ubuntu 22.04 on Raspberry Pi)
# Scope: OS config only. No robot code, no ROS nodes, no sensors.
#
# What this does:
# - Set default to multi-user.target (headless)
# - Disable common display managers (if present)
# - Mask Bluetooth
# - Mask PipeWire + WirePlumber (system + user)
# - Ensure gpio/i2c/spi groups exist; add user to groups
# - Ensure hdmi_blanking=2 is present in /boot/firmware/config.txt when available
# - Print acceptance checks
#
# Safe/idempotent: Yes (re-running should not harm).
#
# Notes:
# - User-level systemd commands require a user session. Run as the normal user (not root).
# - Group membership changes require logout/login or reboot to take effect for the current shell.

die() { echo "ERROR: $*" >&2; exit 1; }

if [[ "${EUID}" -eq 0 ]]; then
  die "Run this script as your normal user (not root). It will sudo as needed."
fi

echo "== Step 1.2: baseline snapshot (pre) =="
hostnamectl || true
systemctl --failed || true

echo
echo "== 1) Headless default target =="
sudo systemctl set-default multi-user.target
# isolate may fail if system forbids; not fatal
sudo systemctl isolate multi-user.target || true
echo "Default target: $(systemctl get-default)"

echo
echo "== 2) Disable common display managers (if installed) =="
for dm in gdm3 lightdm sddm; do
  if systemctl list-unit-files | grep -q "^${dm}\.service"; then
    sudo systemctl disable "${dm}.service" || true
    sudo systemctl stop "${dm}.service" || true
  fi
done

echo
echo "== 3) HDMI blanking (Ubuntu Pi uses /boot/firmware/config.txt) =="
BOOTCFG="/boot/firmware/config.txt"
if [[ -f "${BOOTCFG}" ]]; then
  if grep -qE '^\s*hdmi_blanking=' "${BOOTCFG}"; then
    echo "hdmi_blanking already present:"
    grep -nE '^\s*hdmi_blanking=' "${BOOTCFG}" || true
  else
    echo "Appending hdmi_blanking=2 to ${BOOTCFG}"
    echo "hdmi_blanking=2" | sudo tee -a "${BOOTCFG}" >/dev/null
    grep -nE '^\s*hdmi_blanking=' "${BOOTCFG}" || true
  fi
else
  echo "NOTE: ${BOOTCFG} not found; skipping HDMI blanking."
fi

echo
echo "== 4) Groups: gpio / i2c / spi (create if missing) =="
for g in gpio i2c spi; do
  if ! getent group "${g}" >/dev/null; then
    sudo groupadd "${g}"
    echo "Created group: ${g}"
  else
    echo "Group exists: ${g}"
  fi
done

echo
echo "== 5) Add user to required groups =="
sudo usermod -aG dialout,gpio,i2c,spi,video,input "${USER}"
echo "User groups (may not reflect until re-login/reboot):"
id || true

echo
echo "== 6) Mask Bluetooth completely =="
# bluetooth.socket may not exist on some installs; ignore failures
sudo systemctl stop bluetooth.service bluetooth.socket 2>/dev/null || true
sudo systemctl disable bluetooth.service bluetooth.socket 2>/dev/null || true
sudo systemctl mask bluetooth.service bluetooth.socket 2>/dev/null || true
systemctl status bluetooth.service --no-pager || true

echo
echo "== 7) Mask PipeWire/WirePlumber (system) =="
sudo systemctl stop pipewire pipewire-pulse wireplumber 2>/dev/null || true
sudo systemctl disable pipewire pipewire-pulse wireplumber 2>/dev/null || true
sudo systemctl mask pipewire pipewire-pulse wireplumber 2>/dev/null || true

echo
echo "== 8) Mask PipeWire/WirePlumber (user) =="
# These will fail if user systemd isn't available (rare via non-login shells).
# If that happens, run once from an interactive SSH login (normal user) and rerun.
systemctl --user stop pipewire pipewire-pulse wireplumber 2>/dev/null || true
systemctl --user disable pipewire pipewire-pulse wireplumber 2>/dev/null || true
systemctl --user mask pipewire pipewire-pulse wireplumber 2>/dev/null || true
(systemctl --user list-units | grep -q pipewire && echo "WARNING: pipewire still present in user units") || echo "pipewire: gone (user)"

echo
echo "== Step 1.2: acceptance checks (pre-reboot) =="
uptime || true
systemctl --failed || true
journalctl -p err -b --no-pager | tail -n 60 || true
free -h || true

echo
echo "DONE. Reboot is required to:"
echo "- Apply new group memberships to your session"
echo "- Confirm boot-time silence"
echo
echo "Run: sudo reboot"
