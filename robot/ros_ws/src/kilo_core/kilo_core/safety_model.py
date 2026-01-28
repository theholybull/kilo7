from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, resolve_config_path

ROS_STATE_CONTROL = "/kilo/state/control_json"
ROS_STATE_SAFETY_MODEL = "/kilo/state/safety_model"


class SafetyModel(Node):
    """Phase 4 safety model publisher (contracts + calculations only)."""

    def __init__(self) -> None:
        super().__init__("kilo_safety_model")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)

        self.profile = str(get_cfg(cfg, "safety_model.profile", "normal")).strip().lower()
        self.profiles = get_cfg(cfg, "safety_model.profiles", {}) or {}

        self._latest_control: Optional[Dict[str, Any]] = None

        self.create_subscription(String, ROS_STATE_CONTROL, self._on_control, 10)
        self.pub = self.create_publisher(String, ROS_STATE_SAFETY_MODEL, 10)

        self.create_timer(0.1, self._tick)

    def _on_control(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") != "state_control_v1":
                return
            self._latest_control = obj
        except Exception:
            return

    def _calc(self, speed_mps: float) -> Dict[str, Any]:
        params = self.profiles.get(self.profile, {}) if isinstance(self.profiles, dict) else {}

        reaction_time_s = float(params.get("reaction_time_s", 0.0))
        brake_decel_mps2 = float(params.get("brake_decel_mps2", 0.0))
        buffer_m = float(params.get("buffer_m", 0.0))
        max_speed_mps = float(params.get("max_speed_mps", 0.0))

        model_valid = True
        model_reason = "OK"

        if reaction_time_s <= 0 or brake_decel_mps2 <= 0 or max_speed_mps <= 0:
            model_valid = False
            model_reason = "PARAM_MISSING"

        if speed_mps < 0:
            model_valid = False
            model_reason = "INVALID_INPUT"
            speed_mps = abs(speed_mps)

        if not model_valid:
            return {
                "reaction_time_s": reaction_time_s,
                "brake_decel_mps2": brake_decel_mps2,
                "buffer_m": buffer_m,
                "max_speed_mps": max_speed_mps,
                "stop_distance_m": None,
                "model_valid": False,
                "model_reason": model_reason,
            }

        stop_distance_m = buffer_m + (speed_mps * reaction_time_s) + (speed_mps ** 2) / (2.0 * brake_decel_mps2)

        return {
            "reaction_time_s": reaction_time_s,
            "brake_decel_mps2": brake_decel_mps2,
            "buffer_m": buffer_m,
            "max_speed_mps": max_speed_mps,
            "stop_distance_m": stop_distance_m,
            "model_valid": True,
            "model_reason": model_reason,
        }

    def _tick(self) -> None:
        control = self._latest_control or {}
        applied = control.get("applied", {}) if isinstance(control.get("applied", {}), dict) else {}
        throttle = float(applied.get("throttle", 0.0))

        params = self.profiles.get(self.profile, {}) if isinstance(self.profiles, dict) else {}
        max_speed_mps = float(params.get("max_speed_mps", 0.0))
        speed_mps = abs(throttle) * max_speed_mps

        calc = self._calc(speed_mps)

        obj: Dict[str, Any] = {
            "schema_version": "state_safety_model_v1",
            "ts_ms": now_ts_ms(),
            "profile": self.profile,
            "params": {
                "reaction_time_s": calc["reaction_time_s"],
                "brake_decel_mps2": calc["brake_decel_mps2"],
                "buffer_m": calc["buffer_m"],
            },
            "inputs": {"speed_mps": speed_mps},
            "outputs": {
                "stop_distance_m": calc["stop_distance_m"],
                "max_safe_speed_mps": calc["max_speed_mps"] if calc["model_valid"] else None,
                "model_valid": calc["model_valid"],
                "model_reason": calc["model_reason"],
            },
        }

        msg = String()
        msg.data = json.dumps(obj, separators=(",", ":"))
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SafetyModel()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
