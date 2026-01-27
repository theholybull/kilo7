# Decisions Ledger (append-only)
2026-01-27
DECISION: Safety Gate enforces IMU staleness via imu_ttl_ms and publishes imu_ok/imu_age_ms; stale IMU denies with COMPONENT_MISSING.
WHY: Phone IMU is a required component for Phase 2; staleness must be explicit robot truth and deny motion.
IMPLICATIONS: Safety Gate subscribes to /kilo/phone/imu_json; control clamps on deny; additive-only fields in state_safety_v1.
REVERSIBLE: yes (tune imu_ttl_ms or disable by setting to 0).

Decisions Ledger entry (add this verbatim)

Decision: ROS 2 is a hard prerequisite for the backend install package
Date: 2026-01-16
Why: Backend is a ROS 2 workspace/package requiring rclpy + colcon; attempting to “install backend first” wastes time and causes false backend debugging.
Impact: BUILD_PLAN adds a prerequisite gate; PROJECT_STATE remains unchanged; future packages assume ROS is present.
Guardrails: Does not change Interface Contracts v1.2, MQTT topics, authority model, or safety semantics.

2026-01-13
DECISION: Switch core platform to ROS 2 (offline-first).
WHY: Viam adds complexity and blocks the desired offline autonomy path.
IMPLICATIONS: Robot runs headless; phone is client only; Viam removed from critical path.
REVERSIBLE: yes (but not planned)

2026-01-13
DECISION: Drive model = Ackermann.
WHY: Matches current chassis (steer servo + ESC/throttle).
IMPLICATIONS: Controllers and nav assumptions match Ackermann kinematics.
REVERSIBLE: no (hardware-driven)

2026-01-13
DECISION: Motion control = PWM on Pi @ 60 Hz + dead-stop on command loss + GPIO relay kill.
WHY: Fastest bring-up while retaining a hard safety cut.
IMPLICATIONS: Control loop must be isolated from perception/nav load.
REVERSIBLE: yes (future MCU offload possible)

2026-01-13
DECISION: State estimation = VIO primary; GPS secondary only when quality is good.
WHY: Indoor GPS is unreliable; VIO+IMU is the core.
IMPLICATIONS: System must gate GPS use by quality.
REVERSIBLE: yes

2026-01-13
DECISION: Perception = OAK-D front (RGB+depth), rear USB cam, rear ToF for reverse safety and recovery.
WHY: Depth for front obstacles; ToF for near-field truth.
IMPLICATIONS: Rear ToF participates in recovery behaviors.
REVERSIBLE: yes

2026-01-13
DECISION: Phone↔robot transport = MQTT (LAN/offline).
WHY: Resilient pub/sub, reconnect-friendly for sensor streams and events.
IMPLICATIONS: Define topic contract; phone publishes IMU/GPS/UI commands.
REVERSIBLE: yes

2026-01-13
DECISION: Mapping = 2D; maps stored on USB; multiple named maps; manual selection + default.
WHY: Stable ops; avoids background mapping load.
IMPLICATIONS: UI supports map selection; mapping is explicit mode.
REVERSIBLE: yes

2026-01-13
DECISION: Docking last; pull-through; confirm contact + charge sensing.
WHY: Not a dependency; add once autonomy is stable.
IMPLICATIONS: Docking routines are gated behind stable nav/safety.
REVERSIBLE: yes
