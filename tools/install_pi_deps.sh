#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update

sudo apt-get install -y \
  git curl jq htop \
  python3 python3-venv python3-pip \
  python3-yaml python3-paho-mqtt \
  mosquitto mosquitto-clients \
  udev usbutils \
  pigpio python3-pigpio

# Ensure pigpio daemon is enabled for pwm.mode=pigpio
sudo systemctl enable --now pigpiod

echo "Deps installed (including pigpio + python3-pigpio) and pigpiod enabled."
echo "ROS 2 install is handled separately per Phase 1.1 rules."
