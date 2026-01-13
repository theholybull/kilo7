### Project Charter — KILO .7 (ROS 2, Offline Autonomous Rover)

**Purpose**  
Build an offline-first, autonomous Ackermann-steered rover that can operate reliably without cloud services, using ROS 2 as the core robotics runtime. The system prioritizes stability, field-tunable safety, and incremental autonomy, with self-charging/docking added last.

**Scope**  
The robot runs headless on a Raspberry Pi. Core capabilities include manual control, assistive obstacle avoidance, autonomous navigation, and robust safety interlocks. A mobile phone serves strictly as a client for sensors (IMU/GPS) and user interface; it is optional and not required for operation. Communication between phone and robot uses a reliable local pub/sub transport (MQTT) over LAN.

**Architecture**  
- **Control:** Ackermann drive via PWM at 60 Hz on the Pi, with immediate dead-stop on signal loss and a GPIO-controlled relay for hard power cut.  
- **Safety:** A single Safety Gate arbitrates all motion. It fuses rollover detection, impact (accel + jerk with context), component loss, and battery state, emitting a definitive allow/deny with reason.  
- **Perception:** Front OAK-D provides RGB + depth directly (no middleware cloud), rear USB camera for visibility, and rear ToF sensors for reverse safety and recovery.  
- **State Estimation:** Visual-Inertial Odometry (VIO) is primary; GPS is secondary and used only when quality is sufficient (typically outdoors).  
- **Navigation & Mapping:** 2D mapping and localization with separate modes. Maps are stored on USB as multiple named environments with a selectable default. Heavy compute (mapping) is opt-in and not always-on.  
- **Comms:** MQTT for resilient, offline pub/sub. Phone publishes sensors and commands; robot publishes health, safety state, telemetry, and alerts.

**Tuning Philosophy**  
Avoid fixed thresholds that break with speed changes. Use a **speed-aware safety model** that scales stopping distance with velocity (buffer + reaction time + braking). Expose field-tunable **profiles** (crawl/normal/sport) and a small set of physical parameters. One authority (Safety Gate) owns stop decisions to prevent “panic stop forever.”

**Operating Modes**  
Manual, Assist (manual with obstacle-aware caps), Autonomy (nav behaviors), Mapping (explicit), and Docking (added last). All modes respect the Safety Gate.

**Milestones**  
1) Stable bring-up (PWM, dead-stop, relay kill, telemetry).  
2) Safety Gate operational (rollover/impact/component loss).  
3) Perception v1 (depth front, ToF rear) + speed-aware stopping.  
4) Mapping/localization with USB map management.  
5) Autonomous navigation and recovery.  
6) Docking/charging (AprilTag, pull-through, contact + charge confirmation).

**Out of Scope (for now)**  
Cloud dependencies, fleet management, continuous background mapping, and docking before autonomy is stable.

**Success Criteria**  
Robot operates offline for extended sessions, degrades safely on faults, remains tunable in the field without regressions, and incrementally gains autonomy without destabilizing core control.
