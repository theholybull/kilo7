from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, resolve_config_path

ROS_STATE_SAFETY = "/kilo/state/safety_json"


class SafetyGate(Node):
    """Phase 2.1 stub Safety Gate publishing state_safety_v1."""

    def __init__(self) -> None:
        super().__init__("kilo_safety_gate")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)

        self.override_required = bool(get_cfg(cfg, "safety.override_required", True))
        self.allow_clear_while_override = bool(get_cfg(cfg, "safety.allow_clear_while_override", False))

        # Stub: latched stop by default until later wiring
        self.explicit_stop = True

        self.pub = self.create_publisher(String, ROS_STATE_SAFETY, 10)
        self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        latched = bool(self.explicit_stop)

        if self.override_required:
            safe = False
            reason = "OVERRIDE_REQUIRED"
        elif latched:
            safe = False
            reason = "EXPLICIT_STOP"
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
            # additive
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
