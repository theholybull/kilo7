#!/usr/bin/env python3
"""
Step 2.5 Integration Test: Impact detection denies motion and recovers with hysteresis.
"""

import json
import sys
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kilo_core.util import load_yaml, resolve_config_path, get_cfg

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"
BOLD = "\033[1m"


class Step25ImpactTest(Node):
    def __init__(self) -> None:
        super().__init__("step_2_5_impact_test")
        self.latest_safety: Optional[Dict[str, Any]] = None
        self.latest_control: Optional[Dict[str, Any]] = None

        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)
        self.create_subscription(String, "/kilo/state/control_json", self._on_control, 10)

        self.pub_clear = self.create_publisher(String, "/kilo/cmd/clear_stop_json", 10)
        self.pub_unlock = self.create_publisher(String, "/kilo/cmd/unlock_json", 10)
        self.pub_imu = self.create_publisher(String, "/kilo/phone/imu_json", 10)

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)
        self.impact_thresh = float(get_cfg(cfg, "safety.impact_accel_g_threshold", 2.0))
        self.impact_hysteresis = float(get_cfg(cfg, "safety.impact_hysteresis_g", 0.3))

    def _on_safety(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_safety_v1":
                self.latest_safety = obj
        except Exception:
            pass

    def _on_control(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_control_v1":
                self.latest_control = obj
        except Exception:
            pass

    def _wait_for(self, pred, timeout_s: float = 3.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    def _publish_cmd(self, topic: str, schema: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {"schema_version": schema, "ts_ms": int(time.time() * 1000)},
            separators=(",", ":"),
        )
        if topic == "clear_stop":
            self.pub_clear.publish(msg)
        elif topic == "unlock":
            self.pub_unlock.publish(msg)

    def _publish_imu(self, accel_g: float) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "phone_imu_v1",
                "ts_ms": int(time.time() * 1000),
                "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "accel": {"x": accel_g, "y": 0.0, "z": 0.0},
            },
            separators=(",", ":"),
        )
        self.pub_imu.publish(msg)

    def run(self) -> int:
        print(f"\n{BOLD}Step 2.5 — Impact deny + recovery{RESET}\n")

        self._publish_cmd("unlock", "cmd_unlock_v1")
        self._publish_cmd("clear_stop", "cmd_clear_stop_v1")

        # Publish a spike above threshold
        for _ in range(5):
            self._publish_imu(self.impact_thresh + 0.5)
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self._wait_for(lambda: (self.latest_safety or {}).get("reason") == "IMPACT", 3.0):
            safety = self.latest_safety or {}
            print(f"{RED}✗ FAIL{RESET} expected IMPACT reason")
            print(f"  safety: safe_to_move={safety.get('safe_to_move')}, reason={safety.get('reason')}")
            return 1

        control = self.latest_control or {}
        throttle = control.get("applied", {}).get("throttle", None)
        armed = bool(control.get("armed", True))
        if not (throttle == 0.0 and armed is False):
            print(f"{RED}✗ FAIL{RESET} expected throttle clamp during impact")
            print(f"  control: throttle={throttle}, armed={armed}")
            return 1

        # Publish values below threshold - hysteresis to recover
        recovery_g = max(0.0, self.impact_thresh - self.impact_hysteresis - 0.1)
        for _ in range(5):
            self._publish_imu(recovery_g)
            rclpy.spin_once(self, timeout_sec=0.1)

        recover_reason = {"OK", "OVERRIDE_REQUIRED"}
        if not self._wait_for(lambda: (self.latest_safety or {}).get("reason") in recover_reason, 5.0):
            safety = self.latest_safety or {}
            print(f"{RED}✗ FAIL{RESET} expected recovery to OK/OVERRIDE_REQUIRED")
            print(f"  safety: safe_to_move={safety.get('safe_to_move')}, reason={safety.get('reason')}")
            return 1

        print(f"{GREEN}✓ PASS{RESET} impact deny and recovery observed")
        return 0


def main() -> int:
    rclpy.init()
    node = Step25ImpactTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
