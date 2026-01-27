from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, resolve_config_path

ROS_HW_RELAY_STATUS = "/kilo/hw/relay_status_json"


def _env_int(name: str, default: int) -> int:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    return int(v.strip())


def _env_bool01(name: str, default: bool) -> bool:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    s = v.strip().lower()
    if s in ("1", "true", "yes", "y", "on"):
        return True
    if s in ("0", "false", "no", "n", "off"):
        return False
    raise ValueError(f"Invalid boolean env {name}={v!r}; expected 0/1/true/false")


class RelayKill(Node):
    """Fail-closed relay controller. Publishes hw status always (even when unavailable)."""

    def __init__(self) -> None:
        super().__init__("kilo_relay_kill")

        # best-effort config load; failure must not stop publishing status
        try:
            cfg_path = resolve_config_path(self, package_name="kilo_core")
            cfg = load_yaml(cfg_path)
        except Exception:
            cfg = {}

        yaml_mode = str(get_cfg(cfg, "hardware.relay_mode", "mock")).strip().lower()
        yaml_pin = int(get_cfg(cfg, "hardware.relay_gpio_pin", 17))
        yaml_kill_level_low = bool(get_cfg(cfg, "hardware.relay_kill_level_low", True))

        self.mode = str(os.getenv("KILO_RELAY_MODE", yaml_mode)).strip().lower()
        self.pin = _env_int("KILO_RELAY_GPIO_PIN", yaml_pin)
        self.kill_level_low = _env_bool01("KILO_RELAY_KILL_LEVEL_LOW", yaml_kill_level_low)

        self.available = False
        self.relay_killed = True
        self.relay_reason = "BOOT_SAFE_DEFAULT"

        self._gpio = None

        self._last_kill_state: Optional[bool] = None
        self._last_kill_reason = ""
        self._last_kill_log_mono = 0.0
        self._min_kill_log_s = 1.0

        # Track last control state
        self._last_control_json = None

        # NOTE: keep this conservative for now: publish always; only drive GPIO in rpi_gpio mode
        if self.mode in ("rpi_gpio", "rpi.gpio"):
            try:
                import RPi.GPIO as GPIO  # type: ignore

                self._gpio = GPIO
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT)
                self._apply_kill(True)
                self.available = True
                self.relay_reason = "KILL_ASSERTED_ON_BOOT"
                self.mode = "rpi_gpio"
            except Exception as e:
                self.available = False
                self.relay_killed = True
                self.relay_reason = f"UNAVAILABLE:{type(e).__name__}"
                self._gpio = None
        elif self.mode == "mock":
            self.available = False
            self.relay_killed = True
            self.relay_reason = "UNAVAILABLE"
        else:
            # any other mode not implemented yet; do not crash, just report unavailable truthfully
            self.available = False
            self.relay_killed = True
            self.relay_reason = "UNAVAILABLE"

        self.pub = self.create_publisher(String, ROS_HW_RELAY_STATUS, 10)
        self.create_timer(0.1, self._tick)

        # Subscribe to control authority
        self.create_subscription(
            String,
            "/kilo/state/control_json",
            self._on_control_json,
            10,
        )

    def _on_control_json(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except Exception:
            return
        self._last_control_json = obj
        # Decide KILL/RUN based on control + safety truth; default to KILL (fail-open on ESC power).
        should_kill = True
        reason = "NO_CONTROL_JSON"
        if obj and self.available:
            gate_ok = bool(obj.get("gate_safe_to_move", False))
            gate_reason = str(obj.get("gate_reason", "UNKNOWN"))
            locked = bool(obj.get("locked", True))
            locked_reason = str(obj.get("locked_reason", ""))

            # Fail-OPEN invariants:
            # - Safety denies motion (gate_ok == False)
            # - Explicit STOP asserted by Safety Gate
            # - Control is locked for any reason (including HEARTBEAT_STALE)
            if not gate_ok:
                should_kill = True
                if gate_reason == "EXPLICIT_STOP":
                    reason = "EXPLICIT_STOP"
                elif gate_reason == "OVERRIDE_REQUIRED":
                    reason = "OVERRIDE_REQUIRED"
                else:
                    reason = "SAFETY_DENY"
            elif gate_reason == "EXPLICIT_STOP":
                should_kill = True
                reason = "EXPLICIT_STOP"
            elif locked:
                should_kill = True
                # If lock explicitly due to relay, keep that reason; otherwise mark fail-open on lock
                reason = "RELAY_KILLED" if locked_reason == "RELAY_KILLED" else "LOCKED_FAIL_OPEN"
            else:
                # Eligible for RUN only when Safety allows and Control is unlocked.
                # Primary release path for bootstrap: previously locked only due to RELAY_KILLED
                should_kill = False
                if locked_reason == "RELAY_KILLED":
                    reason = "SAFE_TO_MOVE_RELEASE"
                # Back-compat path: control explicitly publishes relay_killed: false
                elif not obj.get("relay_killed", True):
                    reason = "CONTROL_AUTHORITY_RUN"
                else:
                    reason = "CONTROL_AUTHORITY_RUN"

        self._apply_kill(should_kill)
        self._log_kill_transition(should_kill, reason)
        self.relay_reason = reason

    def _log_kill_transition(self, kill: bool, reason: str) -> None:
        state = bool(kill)
        reason_str = str(reason)
        now = time.monotonic()

        if self._last_kill_state is None:
            if (now - self._last_kill_log_mono) >= self._min_kill_log_s:
                state_label = "KILL" if state else "RUN"
                self.get_logger().info(f"relay_state={state_label} reason={reason_str}")
                self._last_kill_log_mono = now
            self._last_kill_state = state
            self._last_kill_reason = reason_str
            return

        if state == self._last_kill_state and reason_str == self._last_kill_reason:
            return

        if (now - self._last_kill_log_mono) < self._min_kill_log_s:
            self._last_kill_state = state
            self._last_kill_reason = reason_str
            return

        state_label = "KILL" if state else "RUN"
        self.get_logger().info(f"relay_state={state_label} reason={reason_str}")
        self._last_kill_log_mono = now
        self._last_kill_state = state
        self._last_kill_reason = reason_str

    def _apply_kill(self, kill: bool) -> None:
        self.relay_killed = bool(kill)
        if not self.available or self._gpio is None:
            return

        # kill_level_low means: GPIO LOW asserts KILL.
        if self.kill_level_low:
            level = self._gpio.LOW if kill else self._gpio.HIGH
        else:
            level = self._gpio.HIGH if kill else self._gpio.LOW

        self._gpio.output(self.pin, level)

    def _tick(self) -> None:
        obj: Dict[str, Any] = {
            "schema_version": "hw_relay_v1",
            "ts_ms": now_ts_ms(),
            "available": bool(self.available),
            "relay_killed": bool(self.relay_killed),
            "relay_reason": str(self.relay_reason),
            "pin": int(self.pin),
            "mode": str(self.mode),
        }
        m = String()
        m.data = json.dumps(obj, separators=(",", ":"))
        self.pub.publish(m)


def main() -> None:
    rclpy.init()
    node = RelayKill()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception as e:
        # Suppress shutdown noise when rclpy context is already torn down.
        if "context is not valid" not in str(e):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
