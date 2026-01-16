from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .util import (
    build_alert,
    get_cfg,
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
MQTT_UI_MODE = "kilo/ui/mode_request"
MQTT_UI_EMOTION = "kilo/ui/emotion_request"
MQTT_PHONE_IMU = "kilo/phone/imu"
MQTT_PHONE_GPS = "kilo/phone/gps"


class MqttBridge(Node):
    """MQTT <-> ROS JSON bridge (Phase 1–2)."""

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

        # ROS pubs (internal command fanout)
        self.pub_drive = self.create_publisher(String, ROS_CMD_DRIVE, 10)
        self.pub_stop = self.create_publisher(String, ROS_CMD_STOP, 10)
        self.pub_hb = self.create_publisher(String, ROS_CMD_HEARTBEAT, 10)
        self.pub_unlock = self.create_publisher(String, ROS_CMD_UNLOCK, 10)
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

        self._mqtt.connect_async(self.mqtt_host, self.mqtt_port, self.mqtt_keepalive_s)
        self._mqtt.loop_start()

        # cache + attempt retained health immediately
        self._publish_health_retained(ok=True, error="")

        # periodic health
        self.create_timer(1.0, self._tick_health)

    # ---------------- MQTT inbound ----------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc}")
        with self._mqtt_lock:
            self._mqtt_connected = True

        # REQUIRED MINIMUM: STOP/unlock/mode QoS1 (do not downgrade delivery)
        subs = [
            (MQTT_CMD_DRIVE, 0),
            (MQTT_CMD_STOP, 1),
            (MQTT_CMD_HEARTBEAT, 0),
            (MQTT_CMD_UNLOCK, 1),
            (MQTT_UI_MODE, 1),
            (MQTT_UI_EMOTION, 0),
            (MQTT_PHONE_IMU, 0),
            (MQTT_PHONE_GPS, 0),
        ]
        for topic, qos in subs:
            client.subscribe(topic, qos=qos)

        # Flush cached retained health after link is up
        if self._health_last is not None:
            self._publish_mqtt(MQTT_HEALTH, self._health_last, qos=1, retain=True)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")
        with self._mqtt_lock:
            self._mqtt_connected = False
        self._publish_health_retained(ok=False, error=f"mqtt_disconnect_rc_{rc}")

    def _on_mqtt_msg(self, client, userdata, msg):
        obj, err = parse_json_bytes(msg.payload)
        if obj is None:
            alert = build_alert(
                "ERROR",
                "invalid_json_from_mqtt",
                "MQTT publisher emitted non-JSON payload; dropped",
                extra={"topic": msg.topic, "error": err or "unknown"},
            )
            self._publish_mqtt(MQTT_ALERTS, alert, qos=1, retain=False)
            return

        # Contract: every payload MUST include ts_ms. Do not repair; drop and alert.
        if "ts_ms" not in obj:
            alert = build_alert(
                "ERROR",
                "missing_required_field",
                "Inbound MQTT payload missing required field ts_ms; dropped",
                extra={"topic": msg.topic, "missing": "ts_ms"},
            )
            self._publish_mqtt(MQTT_ALERTS, alert, qos=1, retain=False)
            return

        # Robot MUST NOT compute staleness from incoming ts_ms.
        # Add receipt time additively.
        obj["rx_ts_ms"] = now_ts_ms()

        s = json.dumps(obj, separators=(",", ":"))
        ros_msg = String()
        ros_msg.data = s

        if msg.topic == MQTT_CMD_DRIVE:
            self.pub_drive.publish(ros_msg)
        elif msg.topic == MQTT_CMD_STOP:
            self.pub_stop.publish(ros_msg)
        elif msg.topic == MQTT_CMD_HEARTBEAT:
            self.pub_hb.publish(ros_msg)
        elif msg.topic == MQTT_CMD_UNLOCK:
            self.pub_unlock.publish(ros_msg)
        elif msg.topic == MQTT_UI_MODE:
            self.pub_mode.publish(ros_msg)
        elif msg.topic == MQTT_UI_EMOTION:
            self.pub_emotion.publish(ros_msg)
        elif msg.topic == MQTT_PHONE_IMU:
            self.pub_imu.publish(ros_msg)
        elif msg.topic == MQTT_PHONE_GPS:
            self.pub_gps.publish(ros_msg)

    # ---------------- ROS truth inbound ----------------

    def _publish_mqtt(self, topic: str, obj: Dict[str, Any], *, qos: int, retain: bool) -> None:
        payload = json.dumps(obj, separators=(",", ":"))
        with self._mqtt_lock:
            if not self._mqtt_connected:
                return
        self._mqtt.publish(topic, payload=payload, qos=qos, retain=retain)

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
        self._health_last = obj
        self._publish_mqtt(MQTT_HEALTH, obj, qos=1, retain=True)


def main() -> None:
    rclpy.init()
    node = MqttBridge()
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
