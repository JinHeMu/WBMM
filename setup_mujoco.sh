#!/usr/bin/env bash
# Source this file to make the MuJoCo Python packages installed in
# .python_packages available to the ROS 2 Python nodes.
# Usage:
#   source /home/ras/WBMM/setup_mujoco.sh
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}/.python_packages${PYTHONPATH:+:${PYTHONPATH}}"
echo "[setup_mujoco] Added ${SCRIPT_DIR}/.python_packages to PYTHONPATH"
