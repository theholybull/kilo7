from __future__ import annotations

import json
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import now_ts_ms, resolve_config_path, load_yaml, get_cfg


class PerceptionSummaryNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_summary")

        # Load config
        cfg_path = resolve_config_path(self)
        self.cfg: Dict[str, Any] = load_yaml(cfg_path)

        # Publisher: robot truth summary
        self.pub = self.create_publisher(String, "/kilo/state/perception_json", 10)

        # Timer to publish at a low rate (observability only)
        self.timer = self.create_timer(1.0, self._publish)

        self.get_logger().info(f"PerceptionSummary started with config: {cfg_path}")

    def _publish(self) -> None:
        ts_ms = now_ts_ms()
        # Stubbed perception summary (truth-only observability). Additive fields only.
        # In Phase 3, this node will aggregate perception inputs and publish hazard metrics.
        payload: Dict[str, Any] = {
            "schema_version": "state_perception_v1",
            "ts_ms": ts_ms,
            "stale": False,
            "stale_reason": "",
            # Inputs (stubbed)
            "sources": {
                "oak_d_front": {"ok": True, "latency_ms": 0, "age_ms": 0},
                "rear_cam": {"ok": True, "latency_ms": 0, "age_ms": 0},
            },
            # Outputs (stubbed): placeholders for future hazards/stop metrics
            "hazards": {
                "front_obstacle_distance_m": None,
                "rear_obstacle_distance_m": None,
                "side_clearance_m": None,
                "min_stop_distance_m": None,
                "hazard_level": "UNKNOWN",
                "hazard_reason": "",
            },
            "quality": {
                "confidence": None,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PerceptionSummaryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
