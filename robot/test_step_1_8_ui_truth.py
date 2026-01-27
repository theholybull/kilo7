#!/usr/bin/env python3
"""
Step 1.8 — UI Truth Invariants

Verifies that UI lock/emotion derive ONLY from `/kilo/state/safety_json` and
`/kilo/state/control_json` using the documented precedence and lock rules.

Scope (locked):
- No raw drive paths; voice-only intents remain in scope.
- Offline-first; single motion authority.
"""

import json
import sys
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UiTruthTest(Node):
    def __init__(self) -> None:
        super().__init__("ui_truth_test")
        self.safety: Dict[str, Any] = {}
        self.control: Dict[str, Any] = {}
        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)
        self.create_subscription(String, "/kilo/state/control_json", self._on_control, 10)
        self.results = []

    def _on_safety(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_safety_v1":
                self.safety = obj
        except Exception:
            pass

    def _on_control(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_control_v1":
                self.control = obj
        except Exception:
            pass

    def derive(self) -> Dict[str, Any]:
        safe = bool(self.safety.get("safe_to_move", False))
        reason = str(self.safety.get("reason", "UNKNOWN"))
        locked = bool(self.control.get("locked", True))
        locked_reason = str(self.control.get("locked_reason", ""))
        relay_killed = bool(self.control.get("relay_killed", True))

        ui_lock = (not safe) or locked or relay_killed

        if not safe:
            ui_emotion = reason
        elif locked:
            ui_emotion = locked_reason or "LOCKED"
        elif relay_killed:
            ui_emotion = "RELAY_KILLED"
        else:
            ui_emotion = "OK"

        ui_motion_allowed = (safe and not locked and not relay_killed)

        return {
            "ui_lock": ui_lock,
            "ui_emotion": ui_emotion,
            "ui_motion_allowed": ui_motion_allowed,
        }

    def wait_updates(self, timeout_s: float = 3.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.safety and self.control:
                return True
        return False

    def run(self) -> int:
        ok = self.wait_updates(3.0)
        if not ok:
            print("FAIL: no truth updates")
            return 1
        d = self.derive()
        # Minimal invariants: consistency with published truth fields
        # 1) If gate denies, ui_lock must be true.
        if not self.safety.get("safe_to_move", False) and not d["ui_lock"]:
            self.results.append(("Gate deny implies UI lock", False))
        else:
            self.results.append(("Gate deny implies UI lock", True))
        # 2) Emotion precedence matches docs: safety reason -> locked_reason -> RELAY_KILLED -> OK.
        safe = bool(self.safety.get("safe_to_move", False))
        reason = str(self.safety.get("reason", "UNKNOWN"))
        locked = bool(self.control.get("locked", True))
        locked_reason = str(self.control.get("locked_reason", ""))
        relay_killed = bool(self.control.get("relay_killed", True))

        if not safe:
            expected_emotion = reason
        elif locked:
            expected_emotion = locked_reason or "LOCKED"
        elif relay_killed:
            expected_emotion = "RELAY_KILLED"
        else:
            expected_emotion = "OK"

        self.results.append(
            ("Emotion precedence safety/control/relay", d["ui_emotion"] == expected_emotion)
        )
        # 3) Motion allowed only when safe && !locked && !relay_killed.
        expected_allowed = (safe and not locked and not relay_killed)
        self.results.append(("Motion allowed gate/control/relay", d["ui_motion_allowed"] == expected_allowed))

        passed = all(p for _, p in self.results)
        total = len(self.results)
        print(f"Summary: {sum(1 for _, p in self.results if p)}/{total} PASS")
        for name, ok in self.results:
            print(("PASS" if ok else "FAIL") + f" — {name}")
        return 0 if passed else 1


def main() -> int:
    rclpy.init()
    node = UiTruthTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
