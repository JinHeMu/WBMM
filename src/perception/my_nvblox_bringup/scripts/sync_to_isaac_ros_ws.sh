#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/home/a/workspaces/isaac_ros-dev}"
DESTINATION="${ISAAC_ROS_WS}/src/my_nvblox_bringup"

mkdir -p "${DESTINATION}"
rsync -rlt \
  --exclude '.pytest_cache/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  "${PACKAGE_DIR}/" "${DESTINATION}/"

echo "Synced my_nvblox_bringup to ${DESTINATION}"
echo "Build it inside the Isaac ROS container with:"
echo "  colcon build --symlink-install --packages-select my_nvblox_bringup"
