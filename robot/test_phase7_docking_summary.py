#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from kilo_core.docking_summary import DockingSummaryNode


class DockingSummaryTest(Node):
    def __init__(self) -> None:
        super().__init__("phase7_docking_summary_test")
        self.latest = None
        self.create_subscription(String, "/kilo/state/docking_json", self._on_update, 10)

    def _on_update(self, msg: String) -> None:
        try:
            self.latest = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"parse_error: {e}")


def main() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    dock_node = DockingSummaryNode()
    node = DockingSummaryTest()
    executor.add_node(dock_node)
    executor.add_node(node)
    try:
        ok = False
        start = time.time()
        while time.time() - start < 3.0:
            executor.spin_once(timeout_sec=0.1)
            if isinstance(node.latest, dict):
                ok = True
                break
        assert ok, "No docking_json message received"
        doc = node.latest
        assert doc.get("schema_version") == "state_docking_v1", "Wrong schema_version"
        assert "ts_ms" in doc, "Missing ts_ms"
        assert "target" in doc and isinstance(doc["target"], dict), "Missing target dict"
        assert "approach" in doc and isinstance(doc["approach"], dict), "Missing approach dict"
        assert "contact" in doc and isinstance(doc["contact"], dict), "Missing contact dict"
        print({"result": "PASS", "schema": doc.get("schema_version")})
    finally:
        executor.remove_node(node)
        executor.remove_node(dock_node)
        node.destroy_node()
        dock_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
