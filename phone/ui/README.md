# KILO .7 — UI Truth Monitor

This is a read-only UI that derives lock/emotion strictly from the authoritative ROS truth topics:

- `/kilo/state/safety_json`
- `/kilo/state/control_json`

It connects via rosbridge WebSocket and **never** sends commands.

## Open

Open `index.html` in a browser, or serve the folder with any static file server.

### Connection

The UI connects to `ws://<host>:9090` by default.

You can override with query parameters:

- `?host=kilo-dev.local&port=9090`

## Requirements

- rosbridge WebSocket running on the robot (see [docs/VISUALIZER.md](../../docs/VISUALIZER.md)).
- Truth topics publishing (Safety Gate + Control PWM).

## Notes

- UI emotion precedence is: Safety `reason` → Control `locked_reason` → `RELAY_KILLED` → `OK`.
- UI lock condition: `safe_to_move=false` OR `locked=true` OR `relay_killed=true`.
- Motion allowed only when `safe_to_move && !locked && !relay_killed`.
