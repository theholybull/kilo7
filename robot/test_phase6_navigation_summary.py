#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from kilo_core.navigation_summary import NavigationSummaryNode


class NavigationSummaryTest(Node):
    def __init__(self) -> None:
        super().__init__("phase6_navigation_summary_test")
        self.latest = None
        self.create_subscription(String, "/kilo/state/navigation_json", self._on_update, 10)

    def _on_update(self, msg: String) -> None:
        try:
            self.latest = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"parse_error: {e}")


def main() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    nav_node = NavigationSummaryNode()
    node = NavigationSummaryTest()
    executor.add_node(nav_node)
    executor.add_node(node)
    try:
        ok = False
        start = time.time()
        while time.time() - start < 3.0:
            executor.spin_once(timeout_sec=0.1)
            if isinstance(node.latest, dict):
                ok = True
                break
        assert ok, "No navigation_json message received"
        doc = node.latest
        assert doc.get("schema_version") == "state_navigation_v1", "Wrong schema_version"
        assert "ts_ms" in doc, "Missing ts_ms"
        assert "goal" in doc and isinstance(doc["goal"], dict), "Missing goal dict"
        assert "route" in doc and isinstance(doc["route"], dict), "Missing route dict"
        assert "localization" in doc and isinstance(doc["localization"], dict), "Missing localization dict"
        print({"result": "PASS", "schema": doc.get("schema_version")})
    finally:
        executor.remove_node(node)
        executor.remove_node(nav_node)
        node.destroy_node()
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
