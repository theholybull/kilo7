# KILO .7 — Step 1.2: Base OS Hardening (Headless Pi)

## Purpose
Make the Pi baseline boring and deterministic:
- headless boot
- minimal background services
- no desktop/audio/video daemons probing devices
- predictable runtime for later hardware bring-up

**Scope:** OS config only. No robot code. No ROS nodes. No perception/nav.

## What was changed
### Headless operation
- Default boot target set to `multi-user.target` (non-graphical)

### Disabled / prevented background noise
- **Bluetooth**: masked so it cannot start
- **PipeWire / WirePlumber**: masked at both system + user level to prevent v4l2 probing and desktop stack noise
- Display managers (gdm3/lightdm/sddm) disabled if present

### Boot config (Ubuntu on Pi)
- `/boot/firmware/config.txt`: ensured `hdmi_blanking=2` is present (appended if missing)

### Permissions groundwork (no device logic yet)
- Created groups: `gpio`, `i2c`, `spi` (Ubuntu may not ship these by default)
- Added user to: `dialout,gpio,i2c,spi,video,input`

## How to reproduce
Run the script:

```bash
chmod +x scripts/step_1_2_os_hardening.sh
./scripts/step_1_2_os_hardening.sh
sudo reboot



After reboot, run acceptance checks:

systemctl --failed
journalctl -p err -b --no-pager
uptime
free -h
top -b -n1 | head -20
id




Done When (acceptance)

Pi reboots clean

Headless access stable via SSH

systemctl --failed shows 0 failed units

No recurring log spam from bluetooth/pipewire/v4l2 probing

CPU/memory usage low and predictable at idle

id shows the added groups

Rollback (if needed)

Re-enable Bluetooth:

sudo systemctl unmask bluetooth.service bluetooth.socket

sudo systemctl enable --now bluetooth.service

Re-enable PipeWire (if you later want audio/desktop):

sudo systemctl unmask pipewire pipewire-pulse wireplumber

systemctl --user unmask pipewire pipewire-pulse wireplumber

sudo reboot

Notes

Journal output may still show older boot entries. What matters is:

no failed units

services are not running

errors are not recurring on subsequent boots





