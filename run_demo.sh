#!/usr/bin/env bash
# Helper script to launch the UR5e Isaac Sim demo natively on Ubuntu 22.04 with ROS 2 Humble.
# It isolates the environment from active Conda installations and sources the workspace.

set -e

# Sanitize PATH and remove active conda environment to prevent Python version mismatch
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda' | grep -v 'anaconda' | paste -sd:)
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset PYTHONPATH

# Source native ROS 2 Humble and the workspace
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Starting UR5e Isaac Sim Demo (Ubuntu 22.04 / ROS 2 Humble)..."
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py "$@"
