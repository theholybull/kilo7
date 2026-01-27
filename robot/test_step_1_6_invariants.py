#!/usr/bin/env python3
"""
Step 1.6 Integration Test: Control Arbitration Authority Chain

This test verifies the hard requirements from AUTHORITY_CHAIN.md:
1. Single publisher on each truth topic (safety_json, control_json)
2. Safety Gate denial forces applied.throttle to 0.0 exactly
3. Wrong schema on command topics is dropped (not forwarded)
4. Control enforces Safety Gate authority
5. Relay kill state is respected

USAGE:
    source /opt/ros/humble/setup.bash
    source /opt/kilo7/robot/ros_ws/install/setup.bash
    python3 /opt/kilo7/robot/test_step_1_6_invariants.py

PREREQUISITES:
    - ROS 2 Humble running
    - All kilo_core nodes running (safety_gate, control_pwm, mqtt_bridge, relay_kill)
    - MQTT broker accessible (if mqtt_bridge is running)
"""

import json
import time
import sys
import re
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.task import Future

# ANSI colors for output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"


class Step16TestNode(Node):
    """Test node for Step 1.6 invariants."""

    def __init__(self):
        super().__init__("step_1_6_test")
        self.test_results = []
        self.latest_safety = None
        self.latest_control = None

        # Subscriptions for monitoring
        self.create_subscription(
            String, "/kilo/state/safety_json", self._on_safety_update, 10
        )
        self.create_subscription(
            String, "/kilo/state/control_json", self._on_control_update, 10
        )

        # Publishers for testing
        self.pub_drive = self.create_publisher(String, "/kilo/cmd/drive_json", 10)
        self.pub_stop = self.create_publisher(String, "/kilo/cmd/stop_json", 10)
        self.pub_heartbeat = self.create_publisher(
            String, "/kilo/cmd/heartbeat_json", 10
        )
        self.pub_unlock = self.create_publisher(String, "/kilo/cmd/unlock_json", 10)

    def _on_safety_update(self, msg: String) -> None:
        try:
            self.latest_safety = json.loads(msg.data)
        except Exception as e:
            self.log(f"Failed to parse safety_json: {e}")

    def _on_control_update(self, msg: String) -> None:
        try:
            self.latest_control = json.loads(msg.data)
        except Exception as e:
            self.log(f"Failed to parse control_json: {e}")

    def log(self, msg: str) -> None:
        """Print timestamped log message."""
        self.get_logger().info(msg)

    def result(self, test_name: str, passed: bool, detail: str = "") -> None:
        """Record and print test result."""
        status = f"{GREEN}✓ PASS{RESET}" if passed else f"{RED}✗ FAIL{RESET}"
        detail_str = f" ({detail})" if detail else ""
        print(f"{status} {test_name}{detail_str}")
        self.test_results.append((test_name, passed, detail))

    def spin_once_with_timeout(self, timeout_sec: float = 2.0) -> None:
        """Spin for a limited time."""
        future = Future()

        def done_callback(f):
            future.set_result(None)

        timer = self.create_timer(timeout_sec, done_callback)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        except Exception:
            pass
        finally:
            self.destroy_timer(timer)

    def publish_cmd(self, topic_name: str, obj: Dict[str, Any]) -> None:
        """Publish a command object."""
        msg = String()
        msg.data = json.dumps(obj, separators=(",", ":"))

        if topic_name == "drive":
            self.pub_drive.publish(msg)
        elif topic_name == "stop":
            self.pub_stop.publish(msg)
        elif topic_name == "heartbeat":
            self.pub_heartbeat.publish(msg)
        elif topic_name == "unlock":
            self.pub_unlock.publish(msg)

    def test_single_publisher_safety(self) -> None:
        """INVARIANT A1: Only Safety Gate publishes /kilo/state/safety_json"""
        # Check via ros2 topic info
        import subprocess

        try:
            result = subprocess.run(
                ["ros2", "topic", "info", "/kilo/state/safety_json", "--verbose"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            info = result.stdout

            # Parse publisher count from "Publisher count: N" (ignore subscriber count)
            pub_match = re.search(r"Publisher count:\s*(\d+)", info)
            sub_match = re.search(r"Subscription count:\s*(\d+)", info)
            pub_count = int(pub_match.group(1)) if pub_match else None
            sub_count = int(sub_match.group(1)) if sub_match else None

            if result.returncode == 0 and pub_count == 1:
                self.result(
                    "Single Publisher on /kilo/state/safety_json",
                    True,
                    f"publisher_count=1, subscription_count={sub_count}",
                )
            else:
                self.result(
                    "Single Publisher on /kilo/state/safety_json",
                    False,
                    f"publisher_count={pub_count}, subscription_count={sub_count}, info={info}",
                )
        except Exception as e:
            self.result(
                "Single Publisher on /kilo/state/safety_json",
                False,
                str(e),
            )

    def test_single_publisher_control(self) -> None:
        """INVARIANT A2: Only Control publishes /kilo/state/control_json"""
        import subprocess

        try:
            result = subprocess.run(
                ["ros2", "topic", "info", "/kilo/state/control_json", "--verbose"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            info = result.stdout

            pub_match = re.search(r"Publisher count:\s*(\d+)", info)
            sub_match = re.search(r"Subscription count:\s*(\d+)", info)
            pub_count = int(pub_match.group(1)) if pub_match else None
            sub_count = int(sub_match.group(1)) if sub_match else None

            if result.returncode == 0 and pub_count == 1:
                self.result(
                    "Single Publisher on /kilo/state/control_json",
                    True,
                    f"publisher_count=1, subscription_count={sub_count}",
                )
            else:
                self.result(
                    "Single Publisher on /kilo/state/control_json",
                    False,
                    f"publisher_count={pub_count}, subscription_count={sub_count}, info={info}",
                )
        except Exception as e:
            self.result(
                "Single Publisher on /kilo/state/control_json",
                False,
                str(e),
            )

    def test_gate_deny_clamps_throttle(self) -> None:
        """INVARIANT B1: When Safety Gate denies, applied.throttle = 0.0 exactly"""
        ts_ms = int(time.time() * 1000)

        # Send STOP command (forces gate denial)
        self.publish_cmd("stop", {"schema_version": "cmd_stop_v1", "ts_ms": ts_ms})

        # Wait for state updates
        for _ in range(20):  # 2 seconds with 0.1s sleeps
            self.spin_once_with_timeout(0.1)
            time.sleep(0.1)

        if self.latest_safety is None or self.latest_control is None:
            self.result(
                "Gate Deny → Throttle Zero",
                False,
                "No state updates received",
            )
            return

        safe = self.latest_safety.get("safe_to_move", True)
        throttle = self.latest_control.get("applied", {}).get("throttle", -999)
        armed = self.latest_control.get("armed", True)

        passed = (
            not safe  # Gate should deny
            and throttle == 0.0  # Throttle must be exactly 0.0
            and not armed  # Armed must be False
        )

        detail = f"safe={safe}, throttle={throttle}, armed={armed}"
        self.result("Gate Deny → Throttle Zero", passed, detail)

    def test_schema_mismatch_not_accepted(self) -> None:
        """INVARIANT C1: Wrong schema on /kilo/cmd/drive_json is dropped"""
        ts_ms = int(time.time() * 1000)

        # Get current cmd age
        for _ in range(5):
            self.spin_once_with_timeout(0.1)
            time.sleep(0.1)

        age_before = self.latest_control.get("last_cmd_age_ms", 0)

        # Send drive command with WRONG schema
        self.publish_cmd(
            "drive",
            {
                "schema_version": "WRONG_SCHEMA_v1",  # Wrong!
                "steer": 0.5,
                "throttle": 0.5,
                "ts_ms": ts_ms,
            },
        )

        # Wait and check that cmd age increased (not refreshed)
        time.sleep(0.3)
        for _ in range(5):
            self.spin_once_with_timeout(0.1)

        age_after = self.latest_control.get("last_cmd_age_ms", 0)

        # If schema was wrong and dropped, age should have increased
        # (or control should say stale_cmd=True)
        stale = self.latest_control.get("stale_cmd", False)
        passed = stale  # Should be stale since bad command was dropped

        detail = f"age_before={age_before}, age_after={age_after}, stale={stale}"
        self.result("Schema Mismatch → Dropped", passed, detail)

    def test_control_reads_only_safety_gate_truth(self) -> None:
        """INVARIANT B2: Control uses only /kilo/state/safety_json.safe_to_move"""
        # This is proven by examining code + the gate_deny test above
        # Check that control publishes gate_safe_to_move mirror

        ts_ms = int(time.time() * 1000)

        # Send STOP
        self.publish_cmd("stop", {"schema_version": "cmd_stop_v1", "ts_ms": ts_ms})

        # Wait for updates
        for _ in range(20):
            self.spin_once_with_timeout(0.1)
            time.sleep(0.1)

        if self.latest_safety is None or self.latest_control is None:
            self.result(
                "Control Consumes Safety Truth Only",
                False,
                "No updates",
            )
            return

        safe_gate = self.latest_safety.get("safe_to_move", True)
        gate_cache = self.latest_control.get("gate_safe_to_move", True)

        # Control should mirror what gate says
        passed = safe_gate == gate_cache and not safe_gate  # Both should deny

        detail = f"gate={safe_gate}, cache={gate_cache}"
        self.result("Control Consumes Safety Truth Only", passed, detail)

    def test_relay_state_reflected(self) -> None:
        """INVARIANT B3: Control publishes relay state from hw authority"""
        # Check that control publishes relay_killed and relay_reason fields

        if self.latest_control is None:
            self.result(
                "Relay State Published",
                False,
                "No control state",
            )
            return

        has_relay_killed = "relay_killed" in self.latest_control
        has_relay_reason = "relay_reason" in self.latest_control

        passed = has_relay_killed and has_relay_reason

        detail = f"relay_killed={has_relay_killed}, relay_reason={has_relay_reason}"
        self.result("Relay State Published", passed, detail)

    def run_all_tests(self) -> int:
        """Run all Step 1.6 invariant tests."""
        print(f"\n{BOLD}Step 1.6 Integration Tests — Control Arbitration Authority Chain{RESET}\n")

        print("Testing Invariant A (Single Authority):")
        self.test_single_publisher_safety()
        self.test_single_publisher_control()

        print("\nTesting Invariant B (Control Enforces Authority):")
        self.test_gate_deny_clamps_throttle()
        self.test_control_reads_only_safety_gate_truth()
        self.test_relay_state_reflected()

        print("\nTesting Invariant C (Schema Discipline):")
        self.test_schema_mismatch_not_accepted()

        # Summary
        passed = sum(1 for _, p, _ in self.test_results if p)
        total = len(self.test_results)

        print(f"\n{BOLD}Summary: {passed}/{total} tests passed{RESET}")

        if passed == total:
            print(f"{GREEN}✓ All Step 1.6 invariants verified!{RESET}\n")
            return 0
        else:
            print(f"{RED}✗ Some tests failed{RESET}\n")
            for name, passed, detail in self.test_results:
                if not passed:
                    print(f"  Failed: {name} ({detail})")
            return 1


def main() -> int:
    """Entry point."""
    rclpy.init()
    try:
        node = Step16TestNode()
        return_code = node.run_all_tests()
    finally:
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    sys.exit(main())
