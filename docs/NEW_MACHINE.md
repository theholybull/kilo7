1. Clone repo to /home/kilo/kilo7
2. Read PROJECT_CHARTER.md
3. Read PROJECT_STATE.md
4. Follow NEXT CONCRETE STEP
5. Do not build if state is unclear

Fresh install rebuild (Ubuntu 22.04):
1. Run tools/fresh_build_rebuild.sh (as root)
2. Verify mDNS: ping kilo-dev.local
3. Verify Mosquitto: ss -tln | grep 1883
4. Build result: /opt/kilo7/robot/ros_ws/install exists
5. Start nodes (manual):
	- ros2 run kilo_core safety_gate
	- ros2 run kilo_core relay_kill
	- ros2 run kilo_core mqtt_bridge

Notes:
- Mosquitto must listen on 0.0.0.0:1883
- Relay kill defaults to KILL on boot (safe state)
- control_pwm requires pigpio or noop mode before running tests
