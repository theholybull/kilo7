from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import now_ts_ms, resolve_config_path, load_yaml


class NavigationSummaryNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_summary")

        cfg_path = resolve_config_path(self)
        self.cfg: Dict[str, Any] = load_yaml(cfg_path)

        self.pub = self.create_publisher(String, "/kilo/state/navigation_json", 10)
        self.timer = self.create_timer(1.0, self._publish)

        self.get_logger().info(f"NavigationSummary started with config: {cfg_path}")

    def _publish(self) -> None:
        ts_ms = now_ts_ms()
        payload: Dict[str, Any] = {
            "schema_version": "state_navigation_v1",
            "ts_ms": ts_ms,
            "stale": False,
            "stale_reason": "",
            "mode": "NONE",
            "state": "IDLE",
            "goal": {
                "x_m": None,
                "y_m": None,
                "yaw_rad": None,
                "frame_id": "map",
            },
            "route": {
                "valid": False,
                "age_ms": None,
                "remaining_m": None,
            },
            "localization": {
                "pose_valid": False,
                "age_ms": None,
            },
            "quality": {
                "confidence": None,
                "nav_quality": "UNKNOWN",
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = NavigationSummaryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
