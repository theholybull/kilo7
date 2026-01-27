#!/usr/bin/env python3
"""
Step 2.4 Integration Test: Rollover detection denies motion and recovers with hysteresis.
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


class Step24RolloverTest(Node):
    def __init__(self) -> None:
        super().__init__("step_2_4_rollover_test")
        self.latest_safety: Optional[Dict[str, Any]] = None
        self.latest_control: Optional[Dict[str, Any]] = None

        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)
        self.create_subscription(String, "/kilo/state/control_json", self._on_control, 10)

        self.pub_clear = self.create_publisher(String, "/kilo/cmd/clear_stop_json", 10)
        self.pub_unlock = self.create_publisher(String, "/kilo/cmd/unlock_json", 10)
        self.pub_imu = self.create_publisher(String, "/kilo/phone/imu_json", 10)

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)
        self.rollover_tilt_deg = float(get_cfg(cfg, "safety.rollover_tilt_deg", 45.0))
        self.rollover_hysteresis_deg = float(get_cfg(cfg, "safety.rollover_hysteresis_deg", 5.0))

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

    def _publish_imu(self, x: float, y: float, z: float, w: float) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "phone_imu_v1",
                "ts_ms": int(time.time() * 1000),
                "quaternion": {"x": x, "y": y, "z": z, "w": w},
            },
            separators=(",", ":"),
        )
        self.pub_imu.publish(msg)

    def run(self) -> int:
        print(f"\n{BOLD}Step 2.4 — Rollover deny + recovery{RESET}\n")

        # Clear stop + assert override (if required)
        self._publish_cmd("unlock", "cmd_unlock_v1")
        self._publish_cmd("clear_stop", "cmd_clear_stop_v1")

        # Publish IMU with tilt above threshold (~60 deg pitch)
        for _ in range(3):
            self._publish_imu(0.5, 0.0, 0.0, 0.866)
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self._wait_for(lambda: self.latest_safety is not None and self.latest_control is not None, 3.0):
            print(f"{RED}✗ FAIL{RESET} No safety/control updates received")
            return 1

        # Wait for IMU freshness to register
        if not self._wait_for(lambda: bool((self.latest_safety or {}).get("imu_ok", False)), 3.0):
            safety = self.latest_safety or {}
            print(f"{RED}✗ FAIL{RESET} IMU not marked fresh (imu_ok false)")
            print(f"  safety: imu_ok={safety.get('imu_ok')}, imu_age_ms={safety.get('imu_age_ms')}")
            return 1

        # Wait for rollover deny
        if not self._wait_for(lambda: (self.latest_safety or {}).get("reason") == "ROLLOVER", 3.0):
            safety = self.latest_safety or {}
            print(f"{RED}✗ FAIL{RESET} expected ROLLOVER reason")
            print(f"  safety: safe_to_move={safety.get('safe_to_move')}, reason={safety.get('reason')}")
            return 1

        control = self.latest_control or {}
        throttle = control.get("applied", {}).get("throttle", None)
        armed = bool(control.get("armed", True))
        if not (throttle == 0.0 and armed is False):
            print(f"{RED}✗ FAIL{RESET} expected throttle clamp during rollover")
            print(f"  control: throttle={throttle}, armed={armed}")
            return 1

        # Publish IMU below threshold (~10 deg pitch) to recover
        self._publish_imu(0.087, 0.0, 0.0, 0.996)
        recover_reason = {"OK", "OVERRIDE_REQUIRED"}
        if not self._wait_for(lambda: (self.latest_safety or {}).get("reason") in recover_reason, 5.0):
            safety = self.latest_safety or {}
            print(f"{RED}✗ FAIL{RESET} expected recovery to OK/OVERRIDE_REQUIRED")
            print(f"  safety: safe_to_move={safety.get('safe_to_move')}, reason={safety.get('reason')}")
            return 1

        print(f"{GREEN}✓ PASS{RESET} rollover deny and recovery observed")
        return 0


def main() -> int:
    rclpy.init()
    node = Step24RolloverTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
