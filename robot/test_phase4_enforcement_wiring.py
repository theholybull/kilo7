#!/usr/bin/env python3
"""
Phase 4 Enforcement Wiring (config-gated):
- enabled=false => perception hazards do NOT affect Safety Gate (reason OK)
- enabled=true  => perception hazards cause deny with appropriate reason

Test runs SafetyGate in-process (systemd service stopped during test) to avoid
external STOP commands from MQTT and to ensure config reload per phase.
"""

import json
import subprocess
import sys
import time
from typing import Any, Dict, Optional

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from kilo_core.safety_gate import SafetyGate
from kilo_core.util import load_yaml, resolve_config_path


class Phase4EnforcementTest(Node):
    def __init__(self, executor: SingleThreadedExecutor) -> None:
        super().__init__("phase4_enforcement_test")
        self._executor = executor
        self.latest_safety: Optional[Dict[str, Any]] = None
        self.pub_perception = self.create_publisher(String, "/kilo/state/perception_json", 10)
        self.pub_safety_model = self.create_publisher(String, "/kilo/state/safety_model", 10)
        self.pub_clear = self.create_publisher(String, "/kilo/cmd/clear_stop_json", 10)
        self.pub_unlock = self.create_publisher(String, "/kilo/cmd/unlock_json", 10)
        self.pub_imu = self.create_publisher(String, "/kilo/phone/imu_json", 10)
        self.create_subscription(String, "/kilo/state/safety_json", self._on_safety, 10)

    def _spin_once(self, timeout_sec: float = 0.1) -> None:
        self._executor.spin_once(timeout_sec=timeout_sec)

    def _on_safety(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if obj.get("schema_version") == "state_safety_v1":
                self.latest_safety = obj
        except Exception:
            pass

    def _wait_for(self, pred, timeout_s: float = 3.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self._spin_once(0.1)
            if pred():
                return True
        return False

    def _publish_cmd(self, topic: str, schema: str) -> None:
        msg = String()
        msg.data = json.dumps({"schema_version": schema, "ts_ms": int(time.time() * 1000)}, separators=(",", ":"))
        if topic == "clear_stop":
            self.pub_clear.publish(msg)
        elif topic == "unlock":
            self.pub_unlock.publish(msg)

    def _publish_imu(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "phone_imu_v1",
                "ts_ms": int(time.time() * 1000),
                "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            separators=(",", ":"),
        )
        for _ in range(3):
            self.pub_imu.publish(msg)
            self._spin_once(0.05)

    def _publish_safety_model(self, stop_distance_m: float) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "state_safety_model_v1",
                "ts_ms": int(time.time() * 1000),
                "profile": "normal",
                "params": {"reaction_time_s": 0.3, "brake_decel_mps2": 2.5, "buffer_m": 0.5},
                "inputs": {"speed_mps": 1.0},
                "outputs": {
                    "stop_distance_m": float(stop_distance_m),
                    "max_safe_speed_mps": None,
                    "model_valid": True,
                    "model_reason": "OK",
                },
            },
            separators=(",", ":"),
        )
        for _ in range(2):
            self.pub_safety_model.publish(msg)
            self._spin_once(0.05)

    def _publish_perception(self, hazard_level: str, min_stop_distance_m: Optional[float], stale: bool) -> None:
        msg = String()
        payload = {
            "schema_version": "state_perception_v1",
            "ts_ms": int(time.time() * 1000),
            "stale": bool(stale),
            "stale_reason": "STALE" if stale else "",
            "sources": {"oak_d_front": {"ok": True, "latency_ms": 0, "age_ms": 0}},
            "hazards": {
                "front_obstacle_distance_m": None,
                "rear_obstacle_distance_m": None,
                "side_clearance_m": None,
                "min_stop_distance_m": min_stop_distance_m,
                "hazard_level": hazard_level,
                "hazard_reason": "HAZARD" if hazard_level else "",
            },
            "quality": {"confidence": None},
        }
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub_perception.publish(msg)

    def _await_reason(self, timeout_s: float = 3.0) -> Optional[str]:
        if not self._wait_for(lambda: self.latest_safety is not None, timeout_s):
            return None
        return (self.latest_safety or {}).get("reason", "")

    def _wait_for_reason(self, expected: str, timeout_s: float = 4.0) -> bool:
        def _pred() -> bool:
            return self.latest_safety is not None and self.latest_safety.get("reason") == expected

        return self._wait_for(_pred, timeout_s)

    def _prime_state(self) -> None:
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._publish_cmd("unlock", "cmd_unlock_v1")
            self._publish_cmd("clear_stop", "cmd_clear_stop_v1")
            self._publish_imu()
            if self._wait_for(
                lambda: self.latest_safety is not None and self.latest_safety.get("explicit_stop") is False,
                0.5,
            ):
                return
        self._spin_once(0.1)


def _write_yaml(path: str, cfg: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def _set_enforcement(node: Node, enabled: bool, *, min_level: str, min_stop_buffer_m: float, stale_denies: bool) -> str:
    cfg_path = resolve_config_path(node, package_name="kilo_core")
    cfg = load_yaml(cfg_path)
    safety = cfg.setdefault("safety", {})
    safety["imu_ttl_ms"] = 10000
    enforcement = safety.setdefault("perception_enforcement", {})
    enforcement["enabled"] = bool(enabled)
    enforcement["min_hazard_level"] = str(min_level)
    enforcement["min_stop_buffer_m"] = float(min_stop_buffer_m)
    enforcement["stale_denies"] = bool(stale_denies)
    _write_yaml(cfg_path, cfg)
    return cfg_path


def _set_service_active(name: str, active: bool) -> None:
    cmd = ["sudo", "systemctl", "start" if active else "stop", name]
    subprocess.run(cmd, check=True)
    time.sleep(1)


def _restart_gate(executor: SingleThreadedExecutor, gate: Optional[SafetyGate]) -> SafetyGate:
    if gate is not None:
        executor.remove_node(gate)
        gate.destroy_node()
    gate = SafetyGate()
    executor.add_node(gate)
    time.sleep(0.2)
    return gate


def main() -> int:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = Phase4EnforcementTest(executor)
    executor.add_node(node)

    cfg_path = resolve_config_path(node, package_name="kilo_core")
    bridge_was_active = False
    gate_was_active = False
    gate: Optional[SafetyGate] = None

    try:
        bridge_state = subprocess.run(
            ["systemctl", "is-active", "kilo7-mqtt-bridge"],
            check=False,
            capture_output=True,
            text=True,
        )
        bridge_was_active = bridge_state.stdout.strip() == "active"
        if bridge_was_active:
            _set_service_active("kilo7-mqtt-bridge", False)

        gate_state = subprocess.run(
            ["systemctl", "is-active", "kilo7-safety-gate"],
            check=False,
            capture_output=True,
            text=True,
        )
        gate_was_active = gate_state.stdout.strip() == "active"
        if gate_was_active:
            _set_service_active("kilo7-safety-gate", False)

        with open(cfg_path, "r", encoding="utf-8") as f:
            original_text = f.read()

        # Disabled phase
        _set_enforcement(node, False, min_level="STOP", min_stop_buffer_m=0.0, stale_denies=True)
        gate = _restart_gate(executor, gate)
        node.latest_safety = None
        node._prime_state()
        if not node._wait_for_reason("OK"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: disabled phase did not reach OK (reason={reason})")
            return 1
        node._publish_perception("STOP", 5.0, False)
        if not node._wait_for_reason("OK"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: disabled phase unexpected reason={reason}")
            return 1

        # Enabled phase: hazard level deny
        _set_enforcement(node, True, min_level="STOP", min_stop_buffer_m=0.0, stale_denies=True)
        gate = _restart_gate(executor, gate)
        node.latest_safety = None
        node._prime_state()
        if not node._wait_for_reason("OK"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: enabled phase did not reach OK (reason={reason})")
            return 1
        node._publish_imu()
        node._publish_perception("STOP", 5.0, False)
        if not node._wait_for_reason("PERCEPTION_HAZARD"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: enabled hazard phase expected PERCEPTION_HAZARD, got {reason}")
            return 1

        # Enabled phase: insufficient stop buffer
        node._publish_safety_model(1.0)
        node._publish_imu()
        node._publish_perception("CLEAR", 5.0, False)
        if not node._wait_for_reason("INSUFFICIENT_STOP_BUFFER"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: enabled stop-buffer phase expected INSUFFICIENT_STOP_BUFFER, got {reason}")
            return 1

        # Enabled phase: stale perception
        node._publish_imu()
        node._publish_perception("CLEAR", 0.0, True)
        if not node._wait_for_reason("PERCEPTION_STALE"):
            reason = node._await_reason() or "UNKNOWN"
            print(f"FAIL: enabled stale phase expected PERCEPTION_STALE, got {reason}")
            return 1

        print("PASS: enforcement wiring (disabled/enable hazard/stop-buffer/stale)")
        return 0
    finally:
        try:
            if gate is not None:
                executor.remove_node(gate)
                gate.destroy_node()
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        try:
            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(original_text)
            if gate_was_active:
                _set_service_active("kilo7-safety-gate", True)
            if bridge_was_active:
                _set_service_active("kilo7-mqtt-bridge", True)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
