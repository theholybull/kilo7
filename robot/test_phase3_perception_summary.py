#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionSummaryTest(Node):
    def __init__(self):
        super().__init__("phase3_perception_summary_test")
        self.latest = None
        self.create_subscription(String, "/kilo/state/perception_json", self._on_update, 10)

    def _on_update(self, msg: String):
        try:
            self.latest = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"parse_error: {e}")

    def wait_once(self, timeout_s: float = 3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if isinstance(self.latest, dict):
                return True
        return False


def main():
    rclpy.init()
    node = PerceptionSummaryTest()
    try:
        ok = node.wait_once()
        assert ok, "No perception_json message received"
        doc = node.latest
        assert doc.get("schema_version") == "state_perception_v1", "Wrong schema_version"
        assert "ts_ms" in doc, "Missing ts_ms"
        assert "hazards" in doc and isinstance(doc["hazards"], dict), "Missing hazards dict"
        assert "sources" in doc and isinstance(doc["sources"], dict), "Missing sources dict"
        print({"result": "PASS", "schema": doc.get("schema_version"), "hazards_keys": list(doc["hazards"].keys())})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
