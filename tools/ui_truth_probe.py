#!/usr/bin/env python3
"""
Step 1.8 UI Truth Probe

Derives UI lock/emotion strictly from authoritative topics:
- /kilo/state/safety_json (state_safety_v1)
- /kilo/state/control_json (state_control_v1)

Outputs a compact JSON summary for UI:
- source fields: safe_to_move, reason, latched, override_required;
  locked, locked_reason; relay_killed, relay_reason; applied.throttle
- derived: ui_lock, ui_emotion, ui_motion_allowed

Usage:
  source /opt/ros/humble/setup.bash && source /opt/kilo7/robot/ros_ws/install/setup.bash
  python3 /opt/kilo7/tools/ui_truth_probe.py --once
  python3 /opt/kilo7/tools/ui_truth_probe.py --watch
"""

import argparse
import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UiTruthProbe(Node):
    def __init__(self, once: bool = False):
        super().__init__("ui_truth_probe")
        self.once = once
        self._safety: Dict[str, Any] = {}
        self._control: Dict[str, Any] = {}
        self._printed = False
        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)
        self.create_subscription(String, "/kilo/state/control_json", self._on_control, 10)

    def _on_safety(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_safety_v1":
                self._safety = obj
                self._maybe_print()
        except Exception:
            pass

    def _on_control(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_control_v1":
                self._control = obj
                self._maybe_print()
        except Exception:
            pass

    def _derive_ui(self) -> Dict[str, Any]:
        # Extract authoritative fields
        safe = bool(self._safety.get("safe_to_move", False))
        reason = str(self._safety.get("reason", "UNKNOWN"))
        latched = bool(self._safety.get("latched", False))
        override_required = bool(self._safety.get("override_required", False))

        locked = bool(self._control.get("locked", True))
        locked_reason = str(self._control.get("locked_reason", ""))
        relay_killed = bool(self._control.get("relay_killed", True))
        relay_reason = str(self._control.get("relay_reason", "UNKNOWN"))
        applied = self._control.get("applied", {})
        applied_throttle = float(applied.get("throttle", 0.0))

        # Derived UI lock: lock if gate denies OR control locked OR relay killed
        ui_lock = (not safe) or locked or relay_killed

        # Derived UI emotion: precedence Safety → Control → Relay → OK
        if not safe:
            ui_emotion = reason
        elif locked:
            ui_emotion = locked_reason or "LOCKED"
        elif relay_killed:
            ui_emotion = "RELAY_KILLED"
        else:
            ui_emotion = "OK"

        # Motion allowed flag (intent to allow motion, not actual motion)
        ui_motion_allowed = (safe and not locked and not relay_killed)

        return {
            "safe_to_move": safe,
            "reason": reason,
            "latched": latched,
            "override_required": override_required,
            "locked": locked,
            "locked_reason": locked_reason,
            "relay_killed": relay_killed,
            "relay_reason": relay_reason,
            "applied_throttle": applied_throttle,
            "ui_lock": ui_lock,
            "ui_emotion": ui_emotion,
            "ui_motion_allowed": ui_motion_allowed,
        }

    def _maybe_print(self) -> None:
        # Print when both caches are populated
        if not self._safety or not self._control:
            return
        out = self._derive_ui()
        print(json.dumps(out, separators=(",", ":")))
        if self.once:
            self._printed = True


def main() -> int:
        parser = argparse.ArgumentParser()
        parser.add_argument("--once", action="store_true", help="Print once and exit")
        parser.add_argument("--watch", action="store_true", help="Print continuously as updates arrive")
        args = parser.parse_args()

        once = bool(args.once) and not bool(args.watch)
        rclpy.init()
        node = UiTruthProbe(once=once)
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
                if node.once and node._printed:
                    break
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
