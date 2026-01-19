
CURRENT PHASE:
Phase 1 — Bring-up

CURRENT GOAL:
Step 1.5 — Control arbitration seam (consume Safety Gate truth, force neutral when deny) — logic only, no motion

LAST CONFIRMED WORKING STATE:
Step 1.3 — GPIO/PWM/Relay Validation (PASS)
- PCA9685 verified on I2C bus @0x40
  - ESC/throttle channel = 0
  - Steering channel = 1
  - 50Hz operation confirmed
  - All-channels-off command deterministic
- GPIO17 reserved for ESC kill relay control
- Hardware relay kill-path installed inline with ESC power:
  - Battery+ ? COM, ESC+ ? NO
  - Active-LOW trigger (LOW=RUN, HIGH=KILL)
  - Default boot state = KILL
- Validated:
  - Manual RUN/KILL toggle works
  - Reboot returns to KILL
  - Software crash cannot defeat kill-path

Step 1.4 — Safety Gate bring-up (PASS, ROS authority, logic-only)
- Authority: ROS Safety Gate (`kilo7-safety-gate.service` / `kilo_core/safety_gate`)
- Publishes authoritative safety truth on: `/kilo/state/safety_json`
- Legacy HTTP Safety Gate is forbidden:
  - `kilo-safety-gate.service` is masked and must remain masked
  - No listeners on TCP 8098
- Verified behavior (logic-only, no motion):
  - No heartbeat -> DENY / LOSS_OF_COMMAND (fail-closed)
  - Heartbeat with all OK -> ALLOW / OK
  - Stop latch -> DENY / EXPLICIT_STOP
  - Component fault + heartbeat -> DENY / COMPONENT_MISSING (or equivalent)
  - Priority confirmed:
    - LOSS_OF_COMMAND overrides other faults
    - EXPLICIT_STOP overrides all when set

REPO / BASELINE (Option A — authoritative repo on robot):
- `/opt/kilo7` is a git clone and the runtime/build derive from it
- Auto-pull enabled (ff-only, clean-tree only) via:
  - `kilo7-git-autopull.timer`
  - `/opt/kilo7/tools/kilo7_git_autopull.sh`
- Current baseline commit (example): use `git rev-parse --short HEAD` on the robot for exact value

WHAT IS BROKEN (KNOWN):
- None

WHAT IS UNTESTED:
- Battery-critical deny reason (optional)
- Full end-to-end arbitration assertion that control output is forced neutral whenever Safety Gate denies (Step 1.5)

NEXT CONCRETE STEP:
Step 1.5 — Control arbitration seam (logic-only, no motion)
- Define and verify the authoritative chain:
  Command (MQTT->ROS) ? Safety Gate (ROS) ? Control ? Actuators ? State
- Acceptance checks must prove:
  - When Safety Gate denies, throttle is forced neutral (0.0) and relay remains fail-closed as required
  - No other process/service can claim safety authority
  - UI/consumers cannot misinterpret non-authoritative signals as “safe to move”
