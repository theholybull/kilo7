from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import get_cfg, load_yaml, now_ts_ms, resolve_config_path

ROS_HW_RELAY_STATUS = "/kilo/hw/relay_status_json"


class RelayKill(Node):
    """Phase 1.4 relay kill with fail-loud behavior."""

    def __init__(self) -> None:
        super().__init__("kilo_relay_kill")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        cfg = load_yaml(cfg_path)

        self.mode = str(get_cfg(cfg, "hardware.relay_mode", "mock")).strip().lower()
        self.pin = int(get_cfg(cfg, "hardware.relay_gpio_pin", 17))
        self.kill_level_low = bool(get_cfg(cfg, "hardware.relay_kill_level_low", True))

        self.available = False
        self.relay_killed = True
        self.relay_reason = "BOOT_SAFE_DEFAULT"

        self._gpio = None

        if self.mode == "rpi_gpio":
            try:
                import RPi.GPIO as GPIO  # type: ignore
            except Exception as e:
                raise RuntimeError(f"RPi.GPIO required but unavailable: {e}") from e

            self._gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)

            self._apply_kill(True)
            self.available = True
            self.relay_reason = "KILL_ASSERTED_ON_BOOT"

        elif self.mode == "mock":
            self.available = False
            self.relay_killed = True
            self.relay_reason = "UNAVAILABLE"
        else:
            raise ValueError(f"Unknown hardware.relay_mode: {self.mode}")

        self.pub = self.create_publisher(String, ROS_HW_RELAY_STATUS, 10)
        self.create_timer(0.1, self._tick)

    def _apply_kill(self, kill: bool) -> None:
        self.relay_killed = bool(kill)
        if self._gpio is None:
            return

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
            # additive
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
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
