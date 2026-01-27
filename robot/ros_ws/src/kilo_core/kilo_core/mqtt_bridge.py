from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .util import (
    build_alert,
    get_cfg,
    is_valid_intent_request,
    is_valid_imu_request,
    monotonic_s,
    load_yaml,
    now_ts_ms,
    parse_json_bytes,
    resolve_config_path,
)

# Internal ROS topics (JSON strings)
ROS_CMD_DRIVE = "/kilo/cmd/drive_json"
ROS_CMD_STOP = "/kilo/cmd/stop_json"
ROS_CMD_HEARTBEAT = "/kilo/cmd/heartbeat_json"
ROS_CMD_UNLOCK = "/kilo/cmd/unlock_json"
ROS_CMD_CLEAR_STOP = "/kilo/cmd/clear_stop_json"
ROS_PHONE_IMU = "/kilo/phone/imu_json"
ROS_PHONE_GPS = "/kilo/phone/gps_json"
ROS_UI_MODE = "/kilo/ui/mode_request_json"
ROS_UI_EMOTION = "/kilo/ui/emotion_request_json"

ROS_STATE_CONTROL = "/kilo/state/control_json"
ROS_STATE_SAFETY = "/kilo/state/safety_json"
ROS_STATE_SYSTEM = "/kilo/state/system_json"
ROS_ALERTS = "/kilo/alerts_json"

# MQTT topics (contract)
MQTT_HEALTH = "kilo/health"
MQTT_STATE_CONTROL = "kilo/state/control"
MQTT_STATE_SAFETY = "kilo/state/safety"
MQTT_STATE_SYSTEM = "kilo/state/system"
MQTT_ALERTS = "kilo/alerts"

# inbound (untrusted)
MQTT_CMD_DRIVE = "kilo/cmd/drive"
MQTT_CMD_STOP = "kilo/cmd/stop"
MQTT_CMD_HEARTBEAT = "kilo/cmd/heartbeat"
MQTT_CMD_UNLOCK = "kilo/cmd/unlock"
MQTT_CMD_CLEAR_STOP = "kilo/cmd/clear_stop"
MQTT_CMD_INTENT = "kilo/cmd/intent"
MQTT_UI_MODE = "kilo/ui/mode_request"
MQTT_UI_EMOTION = "kilo/ui/emotion_request"
MQTT_PHONE_IMU = "kilo/phone/imu"
MQTT_PHONE_GPS = "kilo/phone/gps"

# Internal ROS topic for intent (added Step 1.7)
ROS_CMD_INTENT = "/kilo/cmd/intent_json"


# Schema version requirements for critical command topics
# LOCKED: Step 1.6 + 1.7 Authority Chain enforcement
MQTT_SCHEMA_REQUIREMENTS = {
    MQTT_CMD_DRIVE: "cmd_drive_v1",
    MQTT_CMD_STOP: "cmd_stop_v1",
    MQTT_CMD_HEARTBEAT: "cmd_heartbeat_v1",
    MQTT_CMD_UNLOCK: "cmd_unlock_v1",
    MQTT_CMD_CLEAR_STOP: "cmd_clear_stop_v1",
    MQTT_CMD_INTENT: "cmd_intent_v1",
    MQTT_PHONE_IMU: "phone_imu_v1",
}


class MqttBridge(Node):
    """MQTT <-> ROS JSON bridge (Phase 1-2)."""

    @staticmethod
    def _canonical_cmd(payload: Dict[str, Any]) -> Dict[str, Any]:
        """Strip non-schema fields. Enforce canonical command shape."""
        return {
            "schema_version": payload.get("schema_version"),
            "ts_ms": payload.get("ts_ms"),
        }

    def __init__(self) -> None:
        super().__init__("kilo_mqtt_bridge")

        cfg_path = resolve_config_path(self, package_name="kilo_core")
        self.cfg = load_yaml(cfg_path)

        self.build_id = str(get_cfg(self.cfg, "build.id", "KILO7_BACKEND"))
        self.mqtt_host = str(get_cfg(self.cfg, "mqtt.host", "127.0.0.1"))
        self.mqtt_port = int(get_cfg(self.cfg, "mqtt.port", 1883))
        self.mqtt_keepalive_s = int(get_cfg(self.cfg, "mqtt.keepalive_s", 20))

        self._start_ts = time.time()
        self._hostname = socket.gethostname()

        # observability knobs (transport-layer only)
        self._log_drive_every_s = float(get_cfg(self.cfg, "mqtt.log_drive_every_s", 1.0))
        self._last_drive_log_ts = 0.0

        # ROS pubs (internal command fanout)
        self.pub_drive = self.create_publisher(String, ROS_CMD_DRIVE, 10)
        self.pub_stop = self.create_publisher(String, ROS_CMD_STOP, 10)
        self.pub_hb = self.create_publisher(String, ROS_CMD_HEARTBEAT, 10)
        self.pub_unlock = self.create_publisher(String, ROS_CMD_UNLOCK, 10)
        self.pub_clear = self.create_publisher(String, ROS_CMD_CLEAR_STOP, 10)
        self.pub_intent = self.create_publisher(String, ROS_CMD_INTENT, 10)
        self.pub_imu = self.create_publisher(String, ROS_PHONE_IMU, 10)
        self.pub_gps = self.create_publisher(String, ROS_PHONE_GPS, 10)
        self.pub_mode = self.create_publisher(String, ROS_UI_MODE, 10)
        self.pub_emotion = self.create_publisher(String, ROS_UI_EMOTION, 10)

        # ROS subs (truth -> mqtt)
        self.create_subscription(String, ROS_STATE_CONTROL, self._on_state_control, 10)
        self.create_subscription(String, ROS_STATE_SAFETY, self._on_state_safety, 10)
        self.create_subscription(String, ROS_STATE_SYSTEM, self._on_state_system, 10)
        self.create_subscription(String, ROS_ALERTS, self._on_alert, 10)

        # MQTT client
        self._mqtt = mqtt.Client(client_id=f"kilo7_{self._hostname}", clean_session=True)
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_msg
        self._mqtt.on_disconnect = self._on_mqtt_disconnect

        user = str(get_cfg(self.cfg, "mqtt.username", "")).strip()
        pw = str(get_cfg(self.cfg, "mqtt.password", "")).strip()
        if user:
            self._mqtt.username_pw_set(user, pw)

        self._mqtt_lock = threading.Lock()
        self._mqtt_connected = False

        self._health_last: Optional[Dict[str, Any]] = None
        # Observability: count inbound MQTT messages by topic
        self._rx_counts: Dict[str, int] = {}

        # Alert rate limiting (to reduce log spam from repeated bad publishers)
        self._alert_min_interval_s = float(get_cfg(self.cfg, "mqtt.alert_min_interval_s", 2.0))
        self._alert_last_ts: Dict[str, float] = {}

        self._mqtt.connect_async(self.mqtt_host, self.mqtt_port, self.mqtt_keepalive_s)
        self._mqtt.loop_start()

        # cache + attempt retained health immediately
        self._publish_health_retained(ok=True, error="")

        # periodic health
        self.create_timer(1.0, self._tick_health)

    # ---------------- MQTT inbound ----------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc} host={self.mqtt_host} port={self.mqtt_port}")
        with self._mqtt_lock:
            self._mqtt_connected = True

        # REQUIRED MINIMUM: STOP/unlock/mode QoS1 (do not downgrade delivery)
        subs = [
            (MQTT_CMD_DRIVE, 0),
            (MQTT_CMD_STOP, 1),
            (MQTT_CMD_HEARTBEAT, 1),
            (MQTT_CMD_UNLOCK, 1),
            (MQTT_CMD_CLEAR_STOP, 1),
            (MQTT_CMD_INTENT, 1),
            (MQTT_UI_MODE, 1),
            (MQTT_UI_EMOTION, 0),
            (MQTT_PHONE_IMU, 0),
            (MQTT_PHONE_GPS, 0),
        ]
        for topic, qos in subs:
            client.subscribe(topic, qos=qos)

        self.get_logger().info(
            "MQTT subscriptions active: "
            + ",".join([t for (t, _) in subs])
        )

        # Flush cached retained health after link is up
        if self._health_last is not None:
            self._publish_mqtt(MQTT_HEALTH, self._health_last, qos=1, retain=True)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warning(f"MQTT disconnected rc={rc}")
        with self._mqtt_lock:
            self._mqtt_connected = False
        self._publish_health_retained(ok=False, error=f"mqtt_disconnect_rc_{rc}")

    def _on_mqtt_msg(self, client, userdata, msg):
        # Increment RX counter for this topic
        try:
            self._rx_counts[msg.topic] = int(self._rx_counts.get(msg.topic, 0)) + 1
        except Exception:
            pass
        obj, err = parse_json_bytes(msg.payload)
        if obj is None:
            alert = build_alert(
                "ERROR",
                "invalid_json_from_mqtt",
                "MQTT publisher emitted non-JSON payload; dropped",
                extra={"topic": msg.topic, "error": err or "unknown"},
            )
            self._emit_alert(alert)
            return

        # Contract: every payload MUST include ts_ms. Do not repair; drop and alert.
        if "ts_ms" not in obj:
            alert = build_alert(
                "ERROR",
                "missing_required_field",
                "Inbound MQTT payload missing required field ts_ms; dropped",
                extra={"topic": msg.topic, "missing": "ts_ms"},
            )
            self._emit_alert(alert)
            return

        # Contract: critical command topics MUST have correct schema_version.
        # Wrong schema -> drop silently (no TTL refresh in consumer).
        if msg.topic in MQTT_SCHEMA_REQUIREMENTS:
            expected_schema = MQTT_SCHEMA_REQUIREMENTS[msg.topic]
            actual_schema = obj.get("schema_version", "")
            if actual_schema != expected_schema:
                self.get_logger().warn(
                    f"Schema mismatch on {msg.topic}: "
                    f"expected={expected_schema}, got={actual_schema}; dropped"
                )
                # Step 1.7 observability: emit alert for schema mismatch (no TTL refresh still)
                alert = build_alert(
                    "ERROR",
                    "schema_mismatch",
                    "Inbound MQTT payload has wrong schema_version; dropped (no TTL refresh)",
                    extra={
                        "topic": msg.topic,
                        "expected": expected_schema,
                        "got": actual_schema,
                    },
                )
                self._emit_alert(alert)
                return

            # Additional validation for Step 1.7 voice intent and IMU
            if msg.topic == MQTT_CMD_INTENT:
                if not is_valid_intent_request(obj):
                    alert = build_alert(
                        "ERROR",
                        "invalid_intent_request",
                        "Intent request failed validation (missing/invalid required fields); dropped",
                        extra={"topic": msg.topic, "intent": obj.get("intent")},
                    )
                    self._emit_alert(alert)
                    return

            elif msg.topic == MQTT_PHONE_IMU:
                if not is_valid_imu_request(obj):
                    alert = build_alert(
                        "ERROR",
                        "invalid_imu_request",
                        "IMU request failed validation (missing orientation); dropped",
                        extra={"topic": msg.topic},
                    )
                    self._emit_alert(alert)
                    return

        # Robot MUST NOT compute staleness from incoming ts_ms.
        # Add receipt time additively.
        obj["rx_ts_ms"] = now_ts_ms()

        s = json.dumps(obj, separators=(",", ":"))
        ros_msg = String()
        ros_msg.data = s

        # ---- ingress fanout + observability ----
        # Step 1.7 guardrail: optionally drop all MQTT drive requests at the bridge.
        # This enforces the "phone sends only intents + IMU, never raw drive" rule
        # without impacting ROS-local tests that publish to /kilo/cmd/drive_json.
        if msg.topic == MQTT_CMD_DRIVE:
            ignore_drive = bool(get_cfg(self.cfg, "mqtt.ignore_drive_from_mqtt", False))
            if ignore_drive:
                alert = build_alert(
                    "WARN",
                    "drive_from_mqtt_blocked",
                    "Inbound MQTT drive request blocked by config; not forwarded to ROS",
                    extra={"topic": msg.topic},
                )
                self._emit_alert(alert)
                return

        if msg.topic == MQTT_CMD_DRIVE:
            now = time.time()
            if self._log_drive_every_s > 0 and (now - self._last_drive_log_ts) >= self._log_drive_every_s:
                self.get_logger().info("RX MQTT drive (rate-limited); TX ROS /kilo/cmd/drive_json")
                self._last_drive_log_ts = now
            self.pub_drive.publish(ros_msg)

        elif msg.topic == MQTT_CMD_STOP:
            # Always log STOP
            self.get_logger().info(f"RX MQTT stop: {s}")
            clean = self._canonical_cmd(obj)
            ros_msg.data = json.dumps(clean, separators=(",", ":"))
            self.pub_stop.publish(ros_msg)
            self.get_logger().info(f"TX ROS {ROS_CMD_STOP}")

        elif msg.topic == MQTT_CMD_HEARTBEAT:
            clean = self._canonical_cmd(obj)
            ros_msg.data = json.dumps(clean, separators=(",", ":"))
            self.pub_hb.publish(ros_msg)

        elif msg.topic == MQTT_CMD_UNLOCK:
            # Always log UNLOCK
            self.get_logger().info(f"RX MQTT unlock: {s}")
            clean = self._canonical_cmd(obj)
            ros_msg.data = json.dumps(clean, separators=(",", ":"))
            self.pub_unlock.publish(ros_msg)
            self.get_logger().info(f"TX ROS {ROS_CMD_UNLOCK}")

        elif msg.topic == MQTT_CMD_CLEAR_STOP:
            # Explicit operator clear of STOP latch
            self.get_logger().info(f"RX MQTT clear_stop: {s}")
            self.pub_clear.publish(ros_msg)
            self.get_logger().info(f"TX ROS {ROS_CMD_CLEAR_STOP}")

        elif msg.topic == MQTT_CMD_INTENT:
            # Step 1.7: Voice intent (request only, not authority)
            intent_str = obj.get("intent", "")
            self.get_logger().info(f"RX MQTT intent: intent={intent_str}")
            self.pub_intent.publish(ros_msg)
            self.get_logger().info(f"TX ROS {ROS_CMD_INTENT}")

        elif msg.topic == MQTT_UI_MODE:
            self.pub_mode.publish(ros_msg)

        elif msg.topic == MQTT_UI_EMOTION:
            self.pub_emotion.publish(ros_msg)

        elif msg.topic == MQTT_PHONE_IMU:
            # Step 1.7: Phone IMU (untrusted sensor input)
            self.get_logger().info("RX MQTT IMU")
            self.pub_imu.publish(ros_msg)
            self.get_logger().info(f"TX ROS {ROS_PHONE_IMU}")

        elif msg.topic == MQTT_PHONE_GPS:
            self.pub_gps.publish(ros_msg)

        else:
            # If we ever get here, someone is publishing to a topic we didn't subscribe to
            # (or the client was configured elsewhere). Never drop silently.
            alert = build_alert(
                "WARN",
                "unhandled_mqtt_topic",
                "MQTT message received for unhandled topic; dropped",
                extra={"topic": msg.topic},
            )
            self._emit_alert(alert)

    # ---------------- ROS truth inbound ----------------

    def _publish_mqtt(self, topic: str, obj: Dict[str, Any], *, qos: int, retain: bool) -> None:
        payload = json.dumps(obj, separators=(",", ":"))
        with self._mqtt_lock:
            if not self._mqtt_connected:
                return
        self._mqtt.publish(topic, payload=payload, qos=qos, retain=retain)

    def _emit_alert(self, alert_obj: Dict[str, Any]) -> None:
        """Publish an alert to MQTT with simple rate limiting by (type, topic).

        This reduces repeated log spam when a misconfigured publisher floods bad messages.
        """
        try:
            typ = str(alert_obj.get("type", "unknown"))
            topic = str(alert_obj.get("topic", ""))
            key = f"{typ}:{topic}"
            now = monotonic_s()
            last = float(self._alert_last_ts.get(key, 0.0))
            if self._alert_min_interval_s <= 0.0 or (now - last) >= self._alert_min_interval_s:
                self._alert_last_ts[key] = now
                self._publish_mqtt(MQTT_ALERTS, alert_obj, qos=1, retain=False)
            else:
                # Optionally accumulate suppressed counts in health via rx_counts
                self._rx_counts[f"suppressed_alert:{key}"] = int(self._rx_counts.get(f"suppressed_alert:{key}", 0)) + 1
        except Exception:
            # Fallback: try to publish the alert without rate limit
            self._publish_mqtt(MQTT_ALERTS, alert_obj, qos=1, retain=False)

    def _on_state_control(self, msg: String) -> None:
        obj = self._coerce_ros_json_or_alert(msg.data, origin_topic=ROS_STATE_CONTROL)
        if obj is None:
            return
        self._publish_mqtt(MQTT_STATE_CONTROL, obj, qos=1, retain=True)

    def _on_state_safety(self, msg: String) -> None:
        obj = self._coerce_ros_json_or_alert(msg.data, origin_topic=ROS_STATE_SAFETY)
        if obj is None:
            return
        self._publish_mqtt(MQTT_STATE_SAFETY, obj, qos=1, retain=True)

    def _on_state_system(self, msg: String) -> None:
        obj = self._coerce_ros_json_or_alert(msg.data, origin_topic=ROS_STATE_SYSTEM)
        if obj is None:
            return
        self._publish_mqtt(MQTT_STATE_SYSTEM, obj, qos=1, retain=True)

    def _on_alert(self, msg: String) -> None:
        obj = self._coerce_ros_json_or_alert(msg.data, origin_topic=ROS_ALERTS)
        if obj is None:
            return
        self._publish_mqtt(MQTT_ALERTS, obj, qos=1, retain=False)

    def _coerce_ros_json_or_alert(self, txt: str, *, origin_topic: str) -> Optional[Dict[str, Any]]:
        try:
            obj = json.loads(txt)
            if not isinstance(obj, dict):
                raise ValueError("not_object")
            return obj
        except Exception:
            alert = build_alert(
                "ERROR",
                "invalid_json_from_ros",
                "ROS publisher emitted non-JSON payload; coerced to alert",
                extra={"topic": origin_topic},
            )
            self._publish_mqtt(MQTT_ALERTS, alert, qos=1, retain=False)
            return None

    # ---------------- Health ----------------

    def _tick_health(self) -> None:
        self._publish_health_retained(ok=True, error="")

    def _publish_health_retained(self, ok: bool, error: str) -> None:
        uptime = max(0.0, time.time() - self._start_ts)
        obj: Dict[str, Any] = {
            "schema_version": "health_v1",
            "ts_ms": now_ts_ms(),
            "ok": bool(ok),
            "error": str(error),
            "build": self.build_id,
            "uptime_s": float(uptime),
        }
        # Add RX counters (additive-only; consumers ignore unknown fields)
        if isinstance(self._rx_counts, dict) and self._rx_counts:
            obj["rx_counts"] = dict(self._rx_counts)
        self._health_last = obj
        self._publish_mqtt(MQTT_HEALTH, obj, qos=1, retain=True)


def main() -> None:
    rclpy.init()
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        # systemd stop / SIGINT can trigger this; treat as clean exit
        pass
    finally:
        # Cleanly stop MQTT network loop before tearing down ROS context
        try:
            if getattr(node, "_mqtt", None) is not None:
                node._mqtt.loop_stop()
                try:
                    node._mqtt.disconnect()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # avoid "rcl_shutdown already called" noise
            pass


if __name__ == "__main__":
    main()
