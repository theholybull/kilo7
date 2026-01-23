from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory


def now_ts_ms() -> int:
    return int(time.time() * 1000)


def monotonic_s() -> float:
    return time.monotonic()


def resolve_config_path(node, param_name: str = "config", package_name: str = "kilo_core") -> str:
    """Resolve config file path with precedence:
    1) ROS param (string) if set
    2) env var KILO_CONFIG if set
    3) package share: <share>/config/kilo.yaml
    """
    try:
        node.declare_parameter(param_name, "")
    except Exception:
        pass

    p = ""
    try:
        p = str(node.get_parameter(param_name).value or "").strip()
    except Exception:
        p = ""

    if p:
        return p

    env_p = os.getenv("KILO_CONFIG", "").strip()
    if env_p:
        return env_p

    share = get_package_share_directory(package_name)
    return os.path.join(share, "config", "kilo.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config must be a mapping/dict: {path}")
    return data


def get_cfg(cfg: Dict[str, Any], dotted: str, default: Any = None) -> Any:
    cur: Any = cfg
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def parse_json_bytes(payload: bytes) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    try:
        txt = payload.decode("utf-8", errors="strict")
        obj = json.loads(txt)
        if not isinstance(obj, dict):
            return None, "payload_not_object"
        return obj, None
    except Exception as e:
        return None, f"json_decode_error:{type(e).__name__}"


def build_alert(
    severity: str,
    typ: str,
    message: str,
    *,
    ts_ms: Optional[int] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Contract-correct alert_v1 minimal required fields.

    Required: schema_version, ts_ms, severity, type, message
    Extra fields are additive-only.
    """
    out: Dict[str, Any] = {
        "schema_version": "alert_v1",
        "ts_ms": int(ts_ms if ts_ms is not None else now_ts_ms()),
        "severity": severity,
        "type": typ,
        "message": message,
    }
    if extra:
        out.update(extra)
    return out


def is_valid_drive_request(obj: Dict[str, Any]) -> bool:
    """LOCKED contract definition of valid drive request."""
    if obj.get("schema_version") != "cmd_drive_v1":
        return False
    if "ts_ms" not in obj:
        return False
    try:
        steer = float(obj.get("steer"))
        throttle = float(obj.get("throttle"))
    except Exception:
        return False
    if not (-1.0 <= steer <= 1.0):
        return False
    if not (-1.0 <= throttle <= 1.0):
        return False
    return True

def is_valid_intent_request(obj: Dict[str, Any]) -> bool:
    """Step 1.7 voice intent contract validation.
    
    Required:
    - schema_version: "cmd_intent_v1" (exact)
    - ts_ms: integer
    - intent: string (enum: STOP, UNLOCK_REQUEST, SET_MODE, ROAM_START, ROAM_STOP, MAPPING_START, MAPPING_STOP, STATUS)
    
    Optional (additive):
    - args: object
    - utterance_id: string
    - confidence: float [0.0, 1.0]
    """
    if obj.get("schema_version") != "cmd_intent_v1":
        return False
    if "ts_ms" not in obj:
        return False
    if "intent" not in obj:
        return False
    intent = str(obj.get("intent", "")).strip()
    valid_intents = {
        "STOP", "UNLOCK_REQUEST", "SET_MODE", "ROAM_START", "ROAM_STOP",
        "MAPPING_START", "MAPPING_STOP", "STATUS"
    }
    if intent not in valid_intents:
        return False
    return True


def is_valid_imu_request(obj: Dict[str, Any]) -> bool:
    """Step 1.7 phone IMU contract validation.
    
    Required:
    - schema_version: "phone_imu_v1" (exact)
    - ts_ms: integer
    - At least one orientation representation:
      * roll, pitch, yaw (floats)
      * OR quaternion (dict with x, y, z, w)
      * OR accel/gyro (impl-dependent)
    """
    if obj.get("schema_version") != "phone_imu_v1":
        return False
    if "ts_ms" not in obj:
        return False
    
    # At least one orientation representation must be present
    has_euler = all(k in obj for k in ["roll", "pitch", "yaw"])
    has_quat = isinstance(obj.get("quaternion"), dict) and all(
        k in obj.get("quaternion", {}) for k in ["x", "y", "z", "w"]
    )
    has_accel_gyro = "accel" in obj or "gyro" in obj
    
    if not (has_euler or has_quat or has_accel_gyro):
        return False
    
    return True