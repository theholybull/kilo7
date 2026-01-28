from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import now_ts_ms, resolve_config_path, load_yaml


class MappingSummaryNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_summary")

        cfg_path = resolve_config_path(self)
        self.cfg: Dict[str, Any] = load_yaml(cfg_path)

        self.pub = self.create_publisher(String, "/kilo/state/mapping_json", 10)
        self.timer = self.create_timer(1.0, self._publish)

        self.get_logger().info(f"MappingSummary started with config: {cfg_path}")

    def _publish(self) -> None:
        ts_ms = now_ts_ms()
        payload: Dict[str, Any] = {
            "schema_version": "state_mapping_v1",
            "ts_ms": ts_ms,
            "stale": False,
            "stale_reason": "",
            "map": {
                "name": "",
                "id": "",
                "status": "NONE",
                "source": "UNKNOWN",
                "frame_id": "map",
            },
            "localization": {
                "pose": {
                    "x_m": None,
                    "y_m": None,
                    "yaw_rad": None,
                },
                "pose_valid": False,
                "age_ms": None,
            },
            "quality": {
                "confidence": None,
                "localization_quality": "UNKNOWN",
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MappingSummaryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
