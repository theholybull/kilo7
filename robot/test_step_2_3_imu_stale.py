#!/usr/bin/env python3
"""
Step 2.3 Integration Test: IMU staleness denies motion (COMPONENT_MISSING).

Workflow:
- Clear explicit stop and assert override via ROS commands.
  (Safety Gate requires explicit action; override_required may be true.)
- Publish a valid IMU sample to establish freshness.
- Wait for IMU to go stale beyond imu_ttl_ms.
- Verify Safety Gate denies with reason COMPONENT_MISSING and Control clamps throttle.
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


class Step23ImuStaleTest(Node):
    def __init__(self) -> None:
        super().__init__("step_2_3_imu_stale_test")
        self.latest_safety: Optional[Dict[str, Any]] = None
        self.latest_control: Optional[Dict[str, Any]] = None

        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)
        self.create_subscription(String, "/kilo/state/control_json", self._on_control, 10)

        self.pub_clear = self.create_publisher(String, "/kilo/cmd/clear_stop_json", 10)
        self.pub_unlock = self.create_publisher(String, "/kilo/cmd/unlock_json", 10)
        self.pub_imu = self.create_publisher(String, "/kilo/phone/imu_json", 10)

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)
        self.imu_ttl_ms = int(get_cfg(cfg, "safety.imu_ttl_ms", 2000))

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

    def _publish_imu(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "phone_imu_v1",
                "ts_ms": int(time.time() * 1000),
                "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            separators=(",", ":"),
        )
        self.pub_imu.publish(msg)

    def run(self) -> int:
        print(f"\n{BOLD}Step 2.3 — IMU stale denies motion{RESET}\n")

        # Assert override + clear explicit stop
        for _ in range(3):
            self._publish_cmd("unlock", "cmd_unlock_v1")
            self._publish_cmd("clear_stop", "cmd_clear_stop_v1")
            rclpy.spin_once(self, timeout_sec=0.1)

        # Ensure explicit stop latch cleared before testing staleness
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            safety = self.latest_safety or {}
            if safety.get("explicit_stop") is False:
                break
            self._publish_cmd("clear_stop", "cmd_clear_stop_v1")
            self._publish_cmd("unlock", "cmd_unlock_v1")

        # Publish a fresh IMU sample
        self._publish_imu()

        # Wait for safety/control updates
        if not self._wait_for(lambda: self.latest_safety is not None and self.latest_control is not None, 3.0):
            print(f"{RED}✗ FAIL{RESET} No safety/control updates received")
            return 1

        # Wait for IMU to go stale and Safety Gate to publish COMPONENT_MISSING
        deadline = time.time() + max(3.0, (self.imu_ttl_ms / 1000.0) + 1.0)
        reason = ""
        safe_to_move = True
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            safety = self.latest_safety or {}
            reason = str(safety.get("reason", ""))
            safe_to_move = bool(safety.get("safe_to_move", True))
            if (not safe_to_move) and (reason == "COMPONENT_MISSING"):
                break

        safety = self.latest_safety or {}
        control = self.latest_control or {}
        throttle = control.get("applied", {}).get("throttle", None)
        armed = bool(control.get("armed", True))

        ok_reason = (not safe_to_move) and (reason == "COMPONENT_MISSING")
        ok_control = (throttle == 0.0) and (armed is False)

        if ok_reason and ok_control:
            print(f"{GREEN}✓ PASS{RESET} IMU stale => COMPONENT_MISSING and throttle clamp")
            return 0

        print(f"{RED}✗ FAIL{RESET} expected COMPONENT_MISSING + throttle 0.0/armed false")
        print(f"  safety: safe_to_move={safe_to_move}, reason={reason}")
        print(f"  control: throttle={throttle}, armed={armed}")
        return 1


def main() -> int:
    rclpy.init()
    node = Step23ImuStaleTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
