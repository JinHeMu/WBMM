#!/usr/bin/env bash
# Subscribe and record only. This script never starts drivers or enables control.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
if [[ $# -gt 1 || ${1:-} == --help ]]; then
  printf 'Usage: bash %s [new-bag-directory]\n' "$0"
  printf 'Start the telemetry nodes separately; Ctrl+C finishes the bag.\n'
  exit 0
fi

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

BAG_DIR="${1:-/tmp/wbmm_force_readonly_$(date +%Y%m%d_%H%M%S)}"
if [[ -e "$BAG_DIR" ]]; then
  printf 'Bag output must not already exist: %s\n' "$BAG_DIR" >&2
  exit 1
fi
mkdir -p -- "$(dirname -- "$BAG_DIR")"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/wbmm_readonly_ros_logs}"
printf 'Recording only; no drivers, control services, or motion commands are started.\n'
printf 'Bag: %s\nStop with Ctrl+C, then inspect with: ros2 bag info "%s"\n' "$BAG_DIR" "$BAG_DIR"

# Parameter getters only; preserve the effective settings rather than assuming
# that current YAML files describe a running process. Missing nodes do not
# prevent the recorder from waiting for topics that appear later.
PARAM_DIR="${BAG_DIR}.params"
mkdir -p -- "$PARAM_DIR"
for node in force_sensor_processor whole_body_force_control wbmm_mrt_node; do
  if ! timeout 5 ros2 param dump "/${node}" > "${PARAM_DIR}/${node}.yaml" 2> "${PARAM_DIR}/${node}.stderr"; then
    printf 'Could not snapshot /%s; recording will still proceed.\n' "$node" >&2
  fi
done

exec ros2 bag record --max-cache-size 20000000 -o "$BAG_DIR" \
  /fts_broadcaster/wrench \
  /whole_body_force_control/processed_wrench \
  /whole_body_force_control/correction \
  /whole_body_force_control/force_sensor_states \
  /whole_body_force_control/states \
  /mobile_manipulator_ee_target \
  /mobile_manipulator_mpc_observation \
  /mobile_manipulator_mpc_policy \
  /mobile_manipulator_force_execution_state \
  /arm_controller/commands /joint_states /wheel/odometry /cmd_vel \
  /tf /tf_static /rosout /parameter_events
