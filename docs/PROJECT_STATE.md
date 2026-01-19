CURRENT PHASE:
Phase 1 — Bring-up

CURRENT GOAL:
Step 1.4 — Safety Gate bring-up (logic only, no motion)

LAST CONFIRMED WORKING STATE:
Step 1.3 — GPIO/PWM/Relay Validation
- PCA9685 verified on I2C bus @0x40
  - ESC/throttle channel = 0
  - Steering channel = 1
  - 50Hz operation confirmed
  - All-channels-off command deterministic
- GPIO17 reserved for ESC kill relay control
- Hardware relay kill-path installed inline with ESC power:
  - Battery+ → COM, ESC+ → NO
  - Active-LOW trigger (LOW=RUN, HIGH=KILL)
  - Default boot state = KILL
- Validated:
  - Manual RUN/KILL toggle works
  - Reboot returns to KILL
  - Software crash cannot defeat kill-path

Step 1.4 — Safety Gate v1 (logic-only service)
- Service installed to /opt/kilo_safety_gate with venv + Flask
- systemd: kilo-safety-gate.service
- systemd drop-in for import reliability:
  - /etc/systemd/system/kilo-safety-gate.service.d/override.conf
  - PYTHONPATH=/opt/kilo_safety_gate
  - PYTHONUNBUFFERED=1
- Config:
  - /etc/kilo/safety_gate.json: override_required=false
- Verified behavior (no motion):
  - No heartbeat -> DENY / LOSS_OF_COMMAND
  - Heartbeat with all OK -> ALLOW / NONE
  - Stop latch -> DENY / EXPLICIT_STOP
  - Component fault + heartbeat -> DENY / COMPONENT_MISSING
  - Priority confirmed: LOSS_OF_COMMAND overrides component faults; EXPLICIT_STOP overrides all when set

WHAT IS BROKEN (KNOWN):
- None

WHAT IS UNTESTED:
- BATTERY_CRITICAL deny reason (optional)
- Integration seam into main backend/control loop (backend not built yet)

NEXT CONCRETE STEP:
Step 1.5 (or next plan item): Define the main backend/control arbitration seam that consumes gate.allow and forces neutral outputs when allow=false (still no motion until that layer is validated).

