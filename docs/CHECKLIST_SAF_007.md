# SAF-007 Operator Checklist — Perception Enforcement (Config-Gated)

Purpose: Safely enable and validate Phase 4 enforcement wiring that gates motion on perception hazards and stop distance, while preserving single-authority and offline-first guardrails.

## Prerequisites
- Phase 3 truth publishing active: `/kilo/state/perception_json` (`state_perception_v1`) with `stale`, per-source `age_ms`, `hazard_level`, `hazard_reason`, `min_stop_distance_m`.
- Phase 4 safety model truth active: `/kilo/state/safety_model` (`state_safety_model_v1`) with outputs including `stop_distance_m`.
- Safety Gate running as single authority: `/kilo/state/safety_json` (`state_safety_v1`).
- Services: `kilo7-safety-gate`, `kilo7-perception-summary`, `kilo7-safety-model`, `kilo7-control`, `kilo7-mqtt-bridge`.

## Enable (config-gated)
Edit config keys in `robot/ros_ws/src/kilo_core/config/kilo.yaml` under `safety.perception_enforcement`:

```yaml
safety:
  perception_enforcement:
    enabled: true            # default is false
    min_hazard_level: "CAUTION"  # enum: UNKNOWN | CLEAR | CAUTION | STOP
    min_stop_buffer_m: 0.0   # additive buffer when comparing to min_stop_distance_m
    stale_denies: true       # deny when perception truth is stale
```

Restart Safety Gate to load config:

```bash
sudo systemctl restart kilo7-safety-gate.service
systemctl --no-pager status kilo7-safety-gate.service
```

## Verification (quick)
Source environment and echo truth topics once:

```bash
source /opt/ros/humble/setup.bash
source /opt/kilo7/robot/ros_ws/install/setup.bash
ros2 topic echo /kilo/state/perception_json std_msgs/msg/String --once
ros2 topic echo /kilo/state/safety_model std_msgs/msg/String --once
ros2 topic echo /kilo/state/safety_json std_msgs/msg/String --once
```

Expected in `/kilo/state/safety_json` (additive observability fields present):
- `perception_ok: bool`
- `perception_age_ms: int|null`
- `perception_reason: string` ("DISABLED" when `enabled=false`, else one of `PERCEPTION_STALE`, `PERCEPTION_HAZARD`, `INSUFFICIENT_STOP_BUFFER`)
- `stop_distance_m: number|null` (from safety model)

## Trigger Scenarios (optional)
- Hazard Level Deny:
  - Ensure `enabled=true` and `min_hazard_level: "CAUTION"`.
  - Publish a one-shot perception message with `hazards.hazard_level: "STOP"`.
  - Expect `/kilo/state/safety_json.reason == "PERCEPTION_HAZARD"` and `safe_to_move=false`.

```bash
ros2 topic pub -1 /kilo/state/perception_json std_msgs/msg/String \
  "{data: '{\"schema_version\":\"state_perception_v1\",\"ts_ms\":$(date +%s%3N),\"stale\":false,\"hazards\":{\"hazard_level\":\"STOP\"}}'}"
```

- Insufficient Stop Buffer Deny:
  - Ensure safety model publishes `stop_distance_m` and set `min_stop_buffer_m` appropriately.
  - Publish perception with `hazards.min_stop_distance_m` greater than `stop_distance_m + min_stop_buffer_m`.
  - Expect `/kilo/state/safety_json.reason == "INSUFFICIENT_STOP_BUFFER"`.

```bash
ros2 topic echo /kilo/state/safety_model std_msgs/msg/String --once | sed -n '1p'
# Use value from stop_distance_m to craft the perception message
ros2 topic pub -1 /kilo/state/perception_json std_msgs/msg/String \
  "{data: '{\"schema_version\":\"state_perception_v1\",\"ts_ms\":$(date +%s%3N),\"stale\":false,\"hazards\":{\"min_stop_distance_m\": 999.0}}'}"
```

- Stale Deny:
  - With `stale_denies=true`, publish `stale: true` in perception.
  - Expect `/kilo/state/safety_json.reason == "PERCEPTION_STALE"`.

```bash
ros2 topic pub -1 /kilo/state/perception_json std_msgs/msg/String \
  "{data: '{\"schema_version\":\"state_perception_v1\",\"ts_ms\":$(date +%s%3N),\"stale\":true,\"stale_reason\":\"simulated\"}'}"
```

## Audit Artifacts
- One-shot helpers (write to timestamped logs):
  - Safety: `tools/capture_safety_once.sh` → `logs/phase_4/<ts>-safety-once/safety_once.json`
  - Perception: `tools/capture_perception_once.sh` → `logs/phase_3/<ts>-perception-once/perception_once.json`
- SAF-007 PASS run: `logs/phase_4/20260128-113403-saf-007-pass/`

## Rollback
- Set `enabled: false` in `kilo.yaml` and restart Safety Gate:

```bash
sudo systemctl restart kilo7-safety-gate.service
```

## Troubleshooting
- Perception truth missing or stale:
  - Check `kilo7-perception-summary.service` status and `/kilo/state/perception_json` contents.
- Safety model missing:
  - Check `kilo7-safety-model.service` and `/kilo/state/safety_model` outputs.
- Authority checks:
  - Verify single publisher on `/kilo/state/safety_json` and `/kilo/state/control_json`.
  - Confirm relay kill and control clamp behavior unchanged.

## Guardrails (unchanged)
- Safety Gate remains the single motion authority; enforcement is config-gated and additive-only.
- No schema renames/removals; only additive fields.
- Offline-first; no cloud dependencies introduced.
