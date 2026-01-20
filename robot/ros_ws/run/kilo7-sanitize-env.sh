#!/usr/bin/env bash
# KILO7 sanitize env: enforce "install overlay only" runtime semantics.
# Must be SOURCED, not executed:
#   source /opt/kilo7/robot/ros_ws/run/kilo7-sanitize-env.sh

set -e

_sanitize_colon_list_var() {
  local var="$1"
  local val="${!var-}"
  [ -n "$val" ] || return 0

  local IFS=':'
  local -a parts out
  read -r -a parts <<<"$val"

  out=()
  for p in "${parts[@]}"; do
    [ -n "$p" ] || continue
    case "$p" in
      *"/robot/ros_ws/build"*|*"/ros_ws/build"*)
        continue
        ;;
    esac
    out+=("$p")
  done

  local joined=""
  local first=1
  for p in "${out[@]}"; do
    if [ $first -eq 1 ]; then
      joined="$p"
      first=0
    else
      joined="${joined}:$p"
    fi
  done

  export "$var=$joined"
}

_sanitize_colon_list_var PYTHONPATH
_sanitize_colon_list_var AMENT_PREFIX_PATH
_sanitize_colon_list_var COLCON_PREFIX_PATH
_sanitize_colon_list_var CMAKE_PREFIX_PATH
_sanitize_colon_list_var LD_LIBRARY_PATH
_sanitize_colon_list_var PATH

unset -f _sanitize_colon_list_var
