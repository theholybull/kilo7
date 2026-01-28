from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, parse_json_bytes, resolve_config_path

ROS_STATE_SAFETY="/kilo/state/safety_json"

# inbound command topics (JSON strings)
ROS_CMD_STOP = "/kilo/cmd/stop_json"
ROS_CMD_UNLOCK = "/kilo/cmd/unlock_json"
ROS_CMD_CLEAR_STOP = "/kilo/cmd/clear_stop_json"
ROS_PHONE_IMU = "/kilo/phone/imu_json"
ROS_STATE_PERCEPTION = "/kilo/state/perception_json"
ROS_STATE_SAFETY_MODEL = "/kilo/state/safety_model"


class SafetyGate(Node):
    """Phase 2.1 Safety Gate: subscribes to STOP + UNLOCK and publishes state_safety_v1.

    STOP is always honored and latches explicit_stop=True.
    UNLOCK is treated as "override asserted" (latched).
    If allow_clear_while_override=true, UNLOCK may also clear the stop latche.
    """

    def __init__(self) -> None:
        super().__init__("kilo_safety_gate")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)

        self.override_required = bool(get_cfg(cfg, "safety.override_required", True))
        self.allow_clear_while_override = bool(get_cfg(cfg, "safety.allow_clear_while_override", False))
        self.imu_ttl_ms = int(get_cfg(cfg, "safety.imu_ttl_ms", 0))
        self.rollover_tilt_deg = float(get_cfg(cfg, "safety.rollover_tilt_deg", 0.0))
        self.rollover_hysteresis_deg = float(get_cfg(cfg, "safety.rollover_hysteresis_deg", 0.0))
        self.impact_accel_g_threshold = float(get_cfg(cfg, "safety.impact_accel_g_threshold", 0.0))
        self.impact_hysteresis_g = float(get_cfg(cfg, "safety.impact_hysteresis_g", 0.0))
        self.perception_enabled = bool(get_cfg(cfg, "safety.perception_enforcement.enabled", False))
        self.perception_min_hazard_level = self._normalize_hazard_level(
            get_cfg(cfg, "safety.perception_enforcement.min_hazard_level", "CAUTION")
        )
        self.perception_min_stop_buffer_m = float(
            get_cfg(cfg, "safety.perception_enforcement.min_stop_buffer_m", 0.0)
        )
        self.perception_stale_denies = bool(
            get_cfg(cfg, "safety.perception_enforcement.stale_denies", True)
        )

        # Latched state
        self.explicit_stop = True  # safe default on boot
        self._override_asserted = False
        self._last_imu_mono: Optional[float] = None
        self._last_imu_tilt_deg: Optional[float] = None
        self._rollover_latched = False
        self._last_imu_accel_g: Optional[float] = None
        self._impact_latched = False

        self._last_perception_mono: Optional[float] = None
        self._perception_hazard_level = "UNKNOWN"
        self._perception_min_stop_distance_m: Optional[float] = None
        self._perception_stale = False
        self._perception_reason = ""

        self._last_stop_distance_m: Optional[float] = None

        self.create_subscription(String, ROS_CMD_STOP, self._on_stop, 10)
        self.create_subscription(String, ROS_CMD_UNLOCK, self._on_unlock, 10)
        self.create_subscription(String, ROS_CMD_CLEAR_STOP, self._on_clear_stop, 10)
        self.create_subscription(String, ROS_PHONE_IMU, self._on_imu, 10)
        self.create_subscription(String, ROS_STATE_PERCEPTION, self._on_perception, 10)
        self.create_subscription(String, ROS_STATE_SAFETY_MODEL, self._on_safety_model, 10)

        self.pub = self.create_publisher(String, ROS_STATE_SAFETY, 10)
        self.create_timer(0.1, self._tick)

    @staticmethod
    def _normalize_hazard_level(level: Any) -> str:
        raw = str(level or "").strip().upper()
        if raw == "WARN":
            raw = "CAUTION"
        if raw == "":
            raw = "UNKNOWN"
        return raw

    def _parse_cmd(self, raw: str) -> tuple[Optional[Dict[str, Any]], Optional[str]]:
        obj, err = parse_json_bytes(raw.encode("utf-8", errors="replace"))
        if obj is None:
            return None, err or "invalid_json"
        if "ts_ms" not in obj:
            return None, "missing_ts_ms"
        return obj, None

    def _on_stop(self, msg: String) -> None:
        obj, err = self._parse_cmd(msg.data)
        if obj is None:
            self.get_logger().warn(f"STOP dropped: {err}")
            return
        self.explicit_stop = True

    def _on_unlock(self, msg: String) -> None:
        obj, err = self._parse_cmd(msg.data)
        if obj is None:
            self.get_logger().warn(f"UNLOCK dropped: {err}")
            return
        if obj.get("schema_version") != "cmd_unlock_v1":
            self.get_logger().warn("UNLOCK dropped: schema_version mismatch")
            return
        self._override_asserted = True
        if self.allow_clear_while_override:
            self.explicit_stop = False

    def _on_clear_stop(self, msg: String) -> None:
        obj, err = self._parse_cmd(msg.data)
        if obj is None:
            self.get_logger().warn(f"CLEAR_STOP dropped: {err}")
            return
        if obj.get("schema_version") != "cmd_clear_stop_v1":
            self.get_logger().warn("CLEAR_STOP dropped: schema_version mismatch")
            return
        # Explicit operator action to clear EXPLICIT_STOP latch
        self.explicit_stop = False

    def _on_imu(self, msg: String) -> None:
        obj, err = parse_json_bytes(msg.data.encode("utf-8", errors="replace"))
        if obj is None:
            self.get_logger().warn(f"IMU dropped: {err or 'invalid_json'}")
            return
        if obj.get("schema_version") != "phone_imu_v1":
            self.get_logger().warn("IMU dropped: schema_version mismatch")
            return
        if "ts_ms" not in obj:
            self.get_logger().warn("IMU dropped: missing_ts_ms")
            return
        quat = obj.get("quaternion") or {}
        try:
            x = float(quat.get("x", 0.0))
            y = float(quat.get("y", 0.0))
            z = float(quat.get("z", 0.0))
            w = float(quat.get("w", 1.0))
            # Gravity vector (body frame), using quaternion
            gx = 2.0 * (x * z - w * y)
            gy = 2.0 * (w * x + y * z)
            gz = w * w - x * x - y * y + z * z
            gz = max(-1.0, min(1.0, gz))
            tilt_rad = math.acos(gz)
            self._last_imu_tilt_deg = math.degrees(tilt_rad)
        except Exception:
            self.get_logger().warn("IMU dropped: invalid_quaternion")
            return
        accel = obj.get("accel") or {}
        try:
            ax = float(accel.get("x", 0.0))
            ay = float(accel.get("y", 0.0))
            az = float(accel.get("z", 0.0))
            self._last_imu_accel_g = math.sqrt(ax * ax + ay * ay + az * az)
        except Exception:
            self._last_imu_accel_g = None
        self._last_imu_mono = time.monotonic()

    def _on_perception(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "state_perception_v1":
            return
        hazards = obj.get("hazards") if isinstance(obj.get("hazards"), dict) else {}
        self._perception_hazard_level = self._normalize_hazard_level(hazards.get("hazard_level", "UNKNOWN"))
        min_stop = hazards.get("min_stop_distance_m")
        self._perception_min_stop_distance_m = float(min_stop) if isinstance(min_stop, (int, float)) else None
        self._perception_stale = bool(obj.get("stale", False))
        self._perception_reason = str(hazards.get("hazard_reason", "")) or str(obj.get("stale_reason", ""))
        self._last_perception_mono = time.monotonic()

    def _on_safety_model(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "state_safety_model_v1":
            return
        outputs = obj.get("outputs") if isinstance(obj.get("outputs"), dict) else {}
        self._last_stop_distance_m = outputs.get("stop_distance_m")

    def _tick(self) -> None:
        latched = bool(self.explicit_stop)
        now_mono = time.monotonic()
        imu_age_ms: Optional[int] = None
        imu_ok = True
        if self.imu_ttl_ms > 0:
            if self._last_imu_mono is None:
                imu_ok = False
            else:
                imu_age_ms = int((now_mono - self._last_imu_mono) * 1000.0)
                imu_ok = imu_age_ms <= self.imu_ttl_ms

        # Rollover detection (hysteresis)
        if self.rollover_tilt_deg > 0.0 and self._last_imu_tilt_deg is not None:
            if not self._rollover_latched and self._last_imu_tilt_deg >= self.rollover_tilt_deg:
                self._rollover_latched = True
            elif self._rollover_latched and self._last_imu_tilt_deg <= (self.rollover_tilt_deg - self.rollover_hysteresis_deg):
                self._rollover_latched = False
        if self.impact_accel_g_threshold > 0.0 and self._last_imu_accel_g is not None:
            if not self._impact_latched and self._last_imu_accel_g >= self.impact_accel_g_threshold:
                self._impact_latched = True
            elif self._impact_latched and self._last_imu_accel_g <= (self.impact_accel_g_threshold - self.impact_hysteresis_g):
                self._impact_latched = False

        # Perception enforcement (config-gated)
        perception_age_ms: Optional[int] = None
        perception_ok = True
        perception_reason = ""
        if self._last_perception_mono is not None:
            perception_age_ms = int((now_mono - self._last_perception_mono) * 1000.0)
        if self.perception_enabled:
            level_order = {"UNKNOWN": 0, "CLEAR": 1, "CAUTION": 2, "STOP": 3}
            hazard_level = self._perception_hazard_level
            min_level = self.perception_min_hazard_level
            if self._last_perception_mono is None and self.perception_stale_denies:
                perception_ok = False
                perception_reason = "PERCEPTION_STALE"
            elif self._perception_stale and self.perception_stale_denies:
                perception_ok = False
                perception_reason = "PERCEPTION_STALE"
            elif level_order.get(hazard_level, 0) >= level_order.get(min_level, 2):
                perception_ok = False
                perception_reason = "PERCEPTION_HAZARD"
            elif (
                self._perception_min_stop_distance_m is not None
                and self._last_stop_distance_m is not None
                and (self._last_stop_distance_m + self.perception_min_stop_buffer_m) < self._perception_min_stop_distance_m
            ):
                perception_ok = False
                perception_reason = "INSUFFICIENT_STOP_BUFFER"
        else:
            perception_ok = True
            perception_reason = "DISABLED"

        if latched:
            safe = False
            reason = "EXPLICIT_STOP"
        elif self.override_required and not self._override_asserted:
            safe = False
            reason = "OVERRIDE_REQUIRED"
        elif self._rollover_latched:
            safe = False
            reason = "ROLLOVER"
        elif self._impact_latched:
            safe = False
            reason = "IMPACT"
        elif not perception_ok:
            safe = False
            reason = perception_reason
        elif not imu_ok:
            safe = False
            reason = "COMPONENT_MISSING"
        else:
            safe = True
            reason = "OK"

        obj: Dict[str, Any] = {
            "schema_version": "state_safety_v1",
            "ts_ms": now_ts_ms(),
            "safe_to_move": bool(safe),
            "reason": str(reason),
            "latched": bool(latched),
            "override_required": bool(self.override_required),
            "explicit_stop": bool(self.explicit_stop),
            "imu_ok": bool(imu_ok),
            "imu_age_ms": imu_age_ms,
            "imu_tilt_deg": self._last_imu_tilt_deg,
            "rollover_latched": bool(self._rollover_latched),
            "imu_accel_g": self._last_imu_accel_g,
            "impact_latched": bool(self._impact_latched),
            "perception_ok": bool(perception_ok),
            "perception_age_ms": perception_age_ms,
            "perception_reason": perception_reason,
            "stop_distance_m": self._last_stop_distance_m,
        }

        m = String()
        m.data = json.dumps(obj, separators=(",", ":"))
        self.pub.publish(m)


def main() -> None:
    rclpy.init()
    node = SafetyGate()
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
