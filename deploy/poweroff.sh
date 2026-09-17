#!/usr/bin/env bash
# Best-effort JAKA power-off/logout and Tracer CAN shutdown.
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
POWEROFF_LOG="${WBMM_LOG_DIR}/poweroff-$(date +%Y%m%d_%H%M%S).log"

log() { printf '[poweroff] %s\n' "$*"; }
warn() { printf '[poweroff] WARN: %s\n' "$*" >&2; }


source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

run_sudo() {
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf '[dry-run] sudo'; printf ' %q' "$@"; printf '\n'
    return 0
  fi
  sudo "$@"
}

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
  source_setup "${ROS_SETUP}"
fi
if [[ -f "${WBMM_WS}/install/setup.bash" ]]; then
  source_setup "${WBMM_WS}/install/setup.bash"
fi
export ROS_DOMAIN_ID RMW_IMPLEMENTATION

STATUS=0

# ---------------------------------------------------------------------------
# 1. JAKA disable/poweroff/logout.
# ---------------------------------------------------------------------------
if [[ "${DRY_RUN}" == "1" ]]; then
  printf '[dry-run] ros2 run jaka_driver jaka_logout %q\n' "${JAKA_IP}"
else
  if command -v ros2 >/dev/null 2>&1; then
    log "jaka_logout ${JAKA_IP}"
    if ros2 run jaka_driver jaka_logout "${JAKA_IP}" 2>&1 | tee "${POWEROFF_LOG}"; then
      log "JAKA logout OK"
    else
      warn "jaka_logout failed"
      STATUS=1
    fi
  else
    warn "ros2 not found; cannot run jaka_logout"
    STATUS=1
  fi
fi

# ---------------------------------------------------------------------------
# 2. Tracer CAN shutdown.
# ---------------------------------------------------------------------------
if [[ "${DRY_RUN}" == "1" ]]; then
  printf '[dry-run] sudo ip link set %q down\n' "${CAN_IFACE}"
else
  if command -v ip >/dev/null 2>&1 && ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
    log "bringing ${CAN_IFACE} down"
    run_sudo ip link set "${CAN_IFACE}" down || {
      warn "failed to bring ${CAN_IFACE} down"
      STATUS=1
    }
  else
    warn "CAN interface ${CAN_IFACE} not present; skipping"
  fi

  if [[ "${CAN_UNLOAD_MODULE}" == "true" && "${CAN_LOAD_MODULE}" == "true" ]]; then
    log "unloading kernel module ${CAN_SETUP_MODULE}"
    run_sudo modprobe -r "${CAN_SETUP_MODULE}" || {
      warn "failed to unload ${CAN_SETUP_MODULE}"
      STATUS=1
    }
  fi
fi

if [[ ${STATUS} -eq 0 ]]; then
  log "poweroff sequence complete"
else
  warn "poweroff sequence completed with errors"
fi
log "log: ${POWEROFF_LOG}"
exit ${STATUS}
