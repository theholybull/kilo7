from __future__ import annotations

import json
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import (
    clamp,
    get_cfg,
    is_valid_drive_request,
    load_yaml,
    monotonic_s,
    now_ts_ms,
    resolve_config_path,
)

ROS_CMD_DRIVE = "/kilo/cmd/drive_json"
ROS_CMD_STOP = "/kilo/cmd/stop_json"
ROS_CMD_HEARTBEAT = "/kilo/cmd/heartbeat_json"
ROS_CMD_UNLOCK = "/kilo/cmd/unlock_json"

ROS_STATE_SAFETY = "/kilo/state/safety_json"
ROS_HW_RELAY_STATUS = "/kilo/hw/relay_status_json"

ROS_STATE_CONTROL = "/kilo/state/control_json"


def _map_unit_to_pulse_us(x: float, center: int, span: int) -> int:
    return int(round(center + (clamp(x, -1.0, 1.0) * span)))


class ControlPWM(Node):
    """Phase 1.3 real PWM control using pigpio (60Hz)."""

    def __init__(self) -> None:
        super().__init__("kilo_control_pwm")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        self.cfg = load_yaml(cfg_path)

        self.cmd_ttl_ms = int(get_cfg(self.cfg, "control.cmd_ttl_ms", 250))
        self.hb_ttl_ms = int(get_cfg(self.cfg, "control.hb_ttl_ms", 1000))

        self.pwm_mode = str(get_cfg(self.cfg, "pwm.mode", "pigpio")).strip().lower()
        self.steer_pin = int(get_cfg(self.cfg, "pwm.steer_gpio_pin", 12))
        self.throttle_pin = int(get_cfg(self.cfg, "pwm.throttle_gpio_pin", 13))

        self.steer_center_us = int(get_cfg(self.cfg, "pwm.steer_center_us", 1500))
        self.steer_span_us = int(get_cfg(self.cfg, "pwm.steer_span_us", 500))
        self.throttle_center_us = int(get_cfg(self.cfg, "pwm.throttle_center_us", 1500))
        self.throttle_span_us = int(get_cfg(self.cfg, "pwm.throttle_span_us", 500))

        self._pi = None
        self._pwm_output_ok = False
        self._applied_simulated = True

        if self.pwm_mode != "pigpio":
            raise RuntimeError("This build is configured for pigpio PWM. Set pwm.mode=pigpio.")

        try:
            import pigpio  # type: ignore
        except Exception as e:
            raise RuntimeError(f"pwm.mode=pigpio requires pigpio python module: {e}") from e

        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError("pigpio daemon not reachable (is pigpiod running?)")

        self._pi = pi
        self._pwm_output_ok = True
        self._applied_simulated = False

        # state
        self._last_valid_cmd_mono: Optional[float] = None
        self._last_hb_mono: Optional[float] = None

        self._requested_steer = 0.0
        self._requested_throttle = 0.0

        self._locked = True
        self._locked_reason = "BOOT_LOCK"
        self._stale_cmd = True
        self._last_cmd_age_ms: int = 0

        self._gate_safe_to_move = False
        self._gate_reason = "UNKNOWN"

        self._relay_killed = True
        self._relay_reason = "UNKNOWN"

        self.create_subscription(String, ROS_CMD_DRIVE, self._on_drive, 10)
        self.create_subscription(String, ROS_CMD_STOP, self._on_stop, 10)
        self.create_subscription(String, ROS_CMD_HEARTBEAT, self._on_heartbeat, 10)
        self.create_subscription(String, ROS_CMD_UNLOCK, self._on_unlock, 10)

        self.create_subscription(String, ROS_STATE_SAFETY, self._on_safety, 10)
        self.create_subscription(String, ROS_HW_RELAY_STATUS, self._on_relay_status, 10)

        self.pub_state = self.create_publisher(String, ROS_STATE_CONTROL, 10)

        self.create_timer(1.0 / 60.0, self._tick)

    def _on_drive(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return

        if not is_valid_drive_request(obj):
            return

        self._requested_steer = clamp(float(obj["steer"]), -1.0, 1.0)
        self._requested_throttle = clamp(float(obj["throttle"]), -1.0, 1.0)

        self._last_valid_cmd_mono = monotonic_s()
        self._stale_cmd = False

    def _on_stop(self, msg: String) -> None:
        self._locked = True
        self._locked_reason = "STOP_REQUEST"
        self._requested_throttle = 0.0

    def _on_heartbeat(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "cmd_heartbeat_v1":
            return
        self._last_hb_mono = monotonic_s()

    def _on_unlock(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "cmd_unlock_v1":
            return

        if self._heartbeat_stale():
            self._locked = True
            self._locked_reason = "HEARTBEAT_STALE"
            return

        self._locked = False
        self._locked_reason = ""

    def _on_safety(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "state_safety_v1":
            return
        self._gate_safe_to_move = bool(obj.get("safe_to_move", False))
        self._gate_reason = str(obj.get("reason", "UNKNOWN"))

    def _on_relay_status(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                return
        except Exception:
            return
        if obj.get("schema_version") != "hw_relay_v1":
            return
        self._relay_killed = bool(obj.get("relay_killed", True))
        self._relay_reason = str(obj.get("relay_reason", "UNKNOWN"))

    def _cmd_stale(self) -> bool:
        if self._last_valid_cmd_mono is None:
            self._last_cmd_age_ms = 0
            return True
        age_ms = int((monotonic_s() - self._last_valid_cmd_mono) * 1000.0)
        self._last_cmd_age_ms = age_ms
        return age_ms > self.cmd_ttl_ms

    def _heartbeat_stale(self) -> bool:
        if self.hb_ttl_ms <= 0:
            return False
        if self._last_hb_mono is None:
            return True
        age_ms = int((monotonic_s() - self._last_hb_mono) * 1000.0)
        return age_ms > self.hb_ttl_ms

    def _write_pwm(self, steer: float, throttle: float) -> Tuple[int, int]:
        steer_us = _map_unit_to_pulse_us(steer, self.steer_center_us, self.steer_span_us)
        thr_us = _map_unit_to_pulse_us(throttle, self.throttle_center_us, self.throttle_span_us)

        assert self._pi is not None
        self._pi.set_servo_pulsewidth(self.steer_pin, steer_us)
        self._pi.set_servo_pulsewidth(self.throttle_pin, thr_us)

        return steer_us, thr_us

    def _tick(self) -> None:
        stale_cmd = self._cmd_stale()
        hb_stale = self._heartbeat_stale()

        if hb_stale:
            self._locked = True
            self._locked_reason = "HEARTBEAT_STALE"

        applied_throttle = self._requested_throttle
        applied_steer = self._requested_steer

        if stale_cmd:
            applied_throttle = 0.0
            self._stale_cmd = True
        else:
            self._stale_cmd = False

        if not self._gate_safe_to_move:
            applied_throttle = 0.0

        if self._locked:
            applied_throttle = 0.0

        if self._relay_killed:
            applied_throttle = 0.0
            if not self._locked:
                self._locked = True
                self._locked_reason = "RELAY_KILLED"

        steer_us, thr_us = self._write_pwm(applied_steer, applied_throttle)

        out: Dict[str, Any] = {
            "schema_version": "state_control_v1",
            "ts_ms": now_ts_ms(),
            "armed": False,
            "locked": bool(self._locked),
            "locked_reason": str(self._locked_reason),
            "stale_cmd": bool(self._stale_cmd),
            "has_cmd": bool(self._last_valid_cmd_mono is not None),
            "last_cmd_age_ms": int(self._last_cmd_age_ms),
            "applied": {"steer": float(applied_steer), "throttle": float(applied_throttle)},
            "relay_killed": bool(self._relay_killed),
            "relay_reason": str(self._relay_reason),
            "pwm_output_ok": bool(self._pwm_output_ok),
            "applied_simulated": bool(self._applied_simulated),
            "applied_pulse_us": {"steer": int(steer_us), "throttle": int(thr_us)},
            "gate_safe_to_move": bool(self._gate_safe_to_move),
            "gate_reason": str(self._gate_reason),
        }

        m = String()
        m.data = json.dumps(out, separators=(",", ":"))
        self.pub_state.publish(m)


def main() -> None:
    rclpy.init()
    node = ControlPWM()
    try:
        rclpy.spin(node)
    finally:
        try:
            if getattr(node, "_pi", None) is not None:
                node._pi.set_servo_pulsewidth(node.steer_pin, 0)
                node._pi.set_servo_pulsewidth(node.throttle_pin, 0)
                node._pi.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
