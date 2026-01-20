#!/usr/bin/env bash
set -eo pipefail

unset PYTHONPATH
export PYTHONNOUSERSITE=1

# ROS setup will reference this var even if unset; don't let bash treat it as fatal
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

source /opt/ros/humble/setup.bash
source "${KILO_ROS_WS:-/opt/kilo7/robot/ros_ws}/install/setup.bash"

CMD=(ros2 run kilo_core control_pwm)

if [ -n "${KILO_CONFIG:-}" ] && [ -f "${KILO_CONFIG}" ]; then
  CMD+=(--ros-args -p "config:=${KILO_CONFIG}")
elif [ -n "${KILO_CONFIG:-}" ]; then
  echo "WARN: KILO_CONFIG set but file not found: ${KILO_CONFIG}. Not passing --ros-args." 1>&2
fi

exec "${CMD[@]}"
