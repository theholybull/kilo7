from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, parse_json_bytes, resolve_config_path

ROS_STATE_SAFETY="/kilo/state/safety_json"

# inbound command topics (JSON strings)
ROS_CMD_STOP = "/kilo/cmd/stop_json"
ROS_CMD_UNLOCK = "/kilo/cmd/unlock_json"


class SafetyGate(Node):
    """Phase 2.1 Safety Gate: subscribes to STOP + UNLOCK and publishes state_safety_v1.

    STOP is always honored and latches explicit_stop=True.
    UNLOCK is treated as "override asserted" (latched).
    If allow_clear_while_override=true, UNLOCK may also clear the stop latche.
    """

    def __init__(self) -> None:
        super().__init__("kilo_safety_gate")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)

        self.override_required = bool(get_cfg(cfg, "safety.override_required", True))
        self.allow_clear_while_override = bool(get_cfg(cfg, "safety.allow_clear_while_override", False))

        # Latched state
        self.explicit_stop = True  # safe default on boot
        self._override_asserted = False

        self.create_subscription(String, ROS_CMD_STOP, self._on_stop, 10)
        self.create_subscription(String, ROS_CMD_UNLOCK, self._on_unlock, 10)

        self.pub = self.create_publisher(String, ROS_STATE_SAFETY, 10)
        self.create_timer(0.1, self._tick)

    def _parse_cmd(self, raw: str) -> tuple[Optional[Dict[str, Any]], Optional[str]]:
        obj, err = parse_json_bytes(raw.encode("utf-8", errors="replace"))
        if obj is None:
            return None, err or "invalid_json"
        if "ts_ms" not in obj:
            return None, "missing_ts_ms"
        return obj, None

    def _on_stop(self, msg: String) -> None:
        obj, err = self._parse_cmd(msg.data)
        if obj is None:
            self.get_logger().warn(f"STOP dropped: {err}")
            return
        self.explicit_stop = True

    def _on_unlock(self, msg: String) -> None:
        obj, err = self._parse_cmd(msg.data)
        if obj is None:
            self.get_logger().warn(f"UNLOCK dropped: {err}")
            return
        if obj.get("schema_version") != "cmd_unlock_v1":
            self.get_logger().warn("UNLOCK dropped: schema_version mismatch")
            return
        self._override_asserted = True
        if self.allow_clear_while_override:
            self.explicit_stop = False

    def _tick(self) -> None:
        latched = bool(self.explicit_stop)

        if latched:
            safe = False
            reason = "EXPLICIT_STOP"
        elif self.override_required and not self._override_asserted:
            safe = False
            reason = "OVERRIDE_REQUIRED"
        else:
            safe = True
            reason = "OK"

        obj: Dict[str, Any] = {
            "schema_version": "state_safety_v1",
            "ts_ms": now_ts_ms(),
            "safe_to_move": bool(safe),
            "reason": str(reason),
            "latched": bool(latched),
            "override_required": bool(self.override_required),
            "explicit_stop": bool(self.explicit_stop),
        }

        m = String()
        m.data = json.dumps(obj, separators=(",", ":"))
        self.pub.publish(m)


def main() -> None:
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
