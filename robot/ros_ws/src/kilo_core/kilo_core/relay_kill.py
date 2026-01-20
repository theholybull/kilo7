from __future__ import annotations

import json
import os
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

        # NOTE: keep this conservative for now: publish always; only drive GPIO in rpi_gpio mode
        if self.mode in ("rpi_gpio", "rpi.gpio"):
            try:
                import RPi.GPIO as GPIO  # type: ignore

                self._gpio = GPIO
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
