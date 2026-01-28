#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionValidityTest(Node):
    def __init__(self):
        super().__init__("phase3_perception_validity_test")
        self.doc = None
        self.create_subscription(String, "/kilo/state/perception_json", self._on_update, 10)

    def _on_update(self, msg: String):
        try:
            self.doc = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"parse_error: {e}")

    def wait_once(self, timeout_s: float = 3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if isinstance(self.doc, dict):
                return True
        return False


def main():
    rclpy.init()
    node = PerceptionValidityTest()
    try:
        ok = node.wait_once()
        assert ok, "No perception_json message received"
        d = node.doc
        # Base fields
        assert d.get("schema_version") == "state_perception_v1", "Wrong schema_version"
        assert isinstance(d.get("ts_ms"), int), "Missing/invalid ts_ms"
        # Staleness
        assert isinstance(d.get("stale"), bool), "stale must be bool"
        assert isinstance(d.get("stale_reason"), str), "stale_reason must be string"
        # Sources
        sources = d.get("sources")
        assert isinstance(sources, dict), "sources must be dict"
        for name in ["oak_d_front", "rear_cam"]:
            s = sources.get(name)
            assert isinstance(s, dict), f"missing source {name}"
            assert isinstance(s.get("ok"), bool), f"{name}.ok must be bool"
            assert isinstance(s.get("latency_ms"), (int, float)), f"{name}.latency_ms must be number"
            assert isinstance(s.get("age_ms"), (int, float)), f"{name}.age_ms must be number"
        # Hazards
        hazards = d.get("hazards")
        assert isinstance(hazards, dict), "hazards must be dict"
        assert "min_stop_distance_m" in hazards, "min_stop_distance_m missing"
        hlevel = hazards.get("hazard_level")
        assert isinstance(hlevel, str), "hazard_level must be string"
        assert isinstance(hazards.get("hazard_reason", ""), str), "hazard_reason must be string"
        # Quality
        quality = d.get("quality")
        assert isinstance(quality, dict), "quality must be dict"
        print({
            "result": "PASS",
            "schema": d.get("schema_version"),
            "stale": d.get("stale"),
            "hazard_level": hlevel,
            "sources": list(sources.keys()),
        })
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
