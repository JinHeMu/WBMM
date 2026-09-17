#!/usr/bin/env bash
# Initialize JAKA and Tracer CAN, then verify both are ready.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${DEPLOY_ENV_FILE:-${SCRIPT_DIR}/env/real.env}"
if [[ ! -f "${ENV_FILE}" ]]; then
  echo "ERROR: deployment env file not found: ${ENV_FILE}" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ENV_FILE}"

DRY_RUN="${DRY_RUN:-0}"
mkdir -p "${WBMM_LOG_DIR}"
START_LOG="${WBMM_LOG_DIR}/start-$(date +%Y%m%d_%H%M%S).log"

log() { printf '[start] %s\n' "$*"; }

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

fail() { printf '[start] ERROR: %s\n' "$*" >&2; exit 1; }

run() {
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf '[dry-run]'; printf ' %q' "$@"; printf '\n'
    return 0
  fi
  "$@"
}

run_sudo() {
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf '[dry-run] sudo'; printf ' %q' "$@"; printf '\n'
    return 0
  fi
  sudo "$@"
}

require_file() {
  [[ -f "$1" ]] || fail "$2: $1"
}

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
require_file "${ROS_SETUP}" "ROS setup file not found"
require_file "${WBMM_WS}/install/setup.bash" "workspace is not built; run deploy/build.sh"

source_setup "${ROS_SETUP}"
source_setup "${WBMM_WS}/install/setup.bash"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION

command -v ip >/dev/null 2>&1 || fail "ip command not found"
command -v ros2 >/dev/null 2>&1 || fail "ros2 command not found"
command -v timeout >/dev/null 2>&1 || fail "timeout command not found"

log "JAKA IP: ${JAKA_IP}"
log "CAN interface: ${CAN_IFACE} @ ${CAN_BITRATE} bit/s"
log "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}, RMW: ${RMW_IMPLEMENTATION}"

# ---------------------------------------------------------------------------
# 1. Bring up the Tracer CAN interface.
# ---------------------------------------------------------------------------
if [[ "${DRY_RUN}" != "1" ]]; then
  if ! sudo -n true 2>/dev/null; then
    log "sudo may ask for a password while configuring ${CAN_IFACE}"
  fi
fi

if [[ "${CAN_LOAD_MODULE}" == "true" ]]; then
  log "loading kernel module: ${CAN_SETUP_MODULE}"
  run_sudo modprobe "${CAN_SETUP_MODULE}" || fail "modprobe ${CAN_SETUP_MODULE} failed"
fi

log "resetting CAN interface ${CAN_IFACE}"
run_sudo ip link set "${CAN_IFACE}" down 2>/dev/null || true
run_sudo ip link set "${CAN_IFACE}" type can bitrate "${CAN_BITRATE}" \
  || fail "cannot configure ${CAN_IFACE} as CAN @ ${CAN_BITRATE}"
run_sudo ip link set "${CAN_IFACE}" up \
  || fail "cannot bring ${CAN_IFACE} up"

if [[ "${DRY_RUN}" != "1" ]]; then
  CAN_DETAILS="$(ip -details link show "${CAN_IFACE}" 2>/dev/null)" \
    || fail "CAN interface ${CAN_IFACE} does not exist after setup"
  grep -qi 'state UP' <<< "${CAN_DETAILS}" \
    || fail "CAN interface ${CAN_IFACE} is not UP"
  grep -qi 'can' <<< "${CAN_DETAILS}" \
    || fail "CAN interface ${CAN_IFACE} is not a CAN device"
  log "CAN validation OK"
  printf '%s\n' "${CAN_DETAILS}" | tee -a "${START_LOG}"
else
  log "DRY_RUN=1, skipping CAN validation"
fi

# ---------------------------------------------------------------------------
# 2. Log in, power on and enable JAKA.
# ---------------------------------------------------------------------------
log "checking jaka_driver executables"
JAKA_EXES="$(ros2 pkg executables jaka_driver 2>/dev/null || true)"
grep -q 'jaka_login' <<< "${JAKA_EXES}" \
  || fail "jaka_driver/jaka_login is not installed; run deploy/build.sh"

JAKA_LOG="$(mktemp)"
log "running: ros2 run jaka_driver jaka_login ${JAKA_IP}"
if [[ "${DRY_RUN}" == "1" ]]; then
  printf '[dry-run] ros2 run jaka_driver jaka_login %q\n' "${JAKA_IP}"
else
  set +e
  ros2 run jaka_driver jaka_login "${JAKA_IP}" 2>&1 | tee "${JAKA_LOG}"
  JAKA_RC=${PIPESTATUS[0]}
  set -e
  cat "${JAKA_LOG}" >> "${START_LOG}"
  [[ ${JAKA_RC} -eq 0 ]] || fail "jaka_login failed with exit code ${JAKA_RC}"
  grep -q 'JAKA communication initialization completed' "${JAKA_LOG}" \
    || fail "jaka_login did not report successful initialization"
  log "JAKA validation OK"
fi
rm -f "${JAKA_LOG}"

log "real hardware initialization complete"
log "log: ${START_LOG}"
