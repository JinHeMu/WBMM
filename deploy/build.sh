#!/usr/bin/env bash
# Build the WBMM bringup and real-robot related packages.
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
mkdir -p "${WBMM_LOG_DIR}" "${WBMM_METADATA_DIR}"
BUILD_LOG="${WBMM_LOG_DIR}/build-$(date +%Y%m%d_%H%M%S).log"

log() { printf '[build] %s\n' "$*"; }

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

fail() { printf '[build] ERROR: %s\n' "$*" >&2; exit 1; }

require_file() {
  [[ -f "$1" ]] || fail "$2: $1"
}

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
require_file "${ROS_SETUP}" "ROS setup file not found"
require_file "${WBMM_WS}/src/bringup/CMakeLists.txt" "WBMM workspace not found"

log "workspace: ${WBMM_WS}"
log "ROS distro: ${ROS_DISTRO}"
log "build type: ${WBMM_BUILD_TYPE}"
log "packages: ${WBMM_BUILD_PACKAGES}"

# Export deployment information next to the build log.
{
  echo "workspace: ${WBMM_WS}"
  echo "date: $(date -Iseconds)"
  echo "ros_distro: ${ROS_DISTRO}"
  echo "build_type: ${WBMM_BUILD_TYPE}"
  if git -C "${WBMM_WS}" rev-parse HEAD >/dev/null 2>&1; then
    echo "git_commit: $(git -C "${WBMM_WS}" rev-parse HEAD)"
    echo "git_dirty: $(git -C "${WBMM_WS}" status --short | wc -l)"
  fi
} > "${WBMM_METADATA_DIR}/deployment_info.txt"

source_setup "${ROS_SETUP}"
if [[ -f "${WBMM_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source_setup "${WBMM_WS}/install/setup.bash"
fi

cd "${WBMM_WS}"

read -r -a PACKAGES <<< "${WBMM_BUILD_PACKAGES}"
COLCON_ARGS=(
  build
  --symlink-install
  --packages-select "${PACKAGES[@]}"
  --cmake-args "-DCMAKE_BUILD_TYPE=${WBMM_BUILD_TYPE}"
  --event-handlers console_direct+
)

log "command: colcon ${COLCON_ARGS[*]}"
if [[ "${DRY_RUN}" == "1" ]]; then
  log "DRY_RUN=1, skipping colcon build"
  exit 0
fi

set -o pipefail
colcon "${COLCON_ARGS[@]}" 2>&1 | tee "${BUILD_LOG}"
source_setup "${WBMM_WS}/install/setup.bash"

log "build succeeded"
log "log: ${BUILD_LOG}"
