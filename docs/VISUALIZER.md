Visualizer Setup (Rosbridge WebSocket)

Purpose: Provide a WebSocket endpoint for visualizers to read ROS truth topics without modifying backend nodes.

Unit Template

- Repo path: `robot/ros_ws/run/systemd/kilo7-rosbridge.service`

Install + Enable

```bash
sudo cp /opt/kilo7/robot/ros_ws/run/systemd/kilo7-rosbridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now kilo7-rosbridge.service
systemctl --no-pager status kilo7-rosbridge.service
ss -ltnp | grep ':9090' || echo '9090 not listening'
```

Connect

- ws://kilo-dev.local:9090 (LAN mDNS)
- ws://<robot-ip>:9090

Notes

- If the `kilo` user does not exist on the target, remove the `User=kilo` line in the unit file.
- Ensure `ros-humble-rosbridge-server` is installed: `sudo apt install ros-humble-rosbridge-server`.
- Address already in use on re-launch is expected if the systemd service is active.

Verification

- `ss -ltnp | grep ':9090'` shows a listener for 9090.
- `journalctl -u kilo7-rosbridge.service -n 60 --no-pager` shows service logs.
