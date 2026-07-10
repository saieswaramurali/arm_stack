#!/usr/bin/env bash
# First-time setup for arm-stack: installs dependencies and builds the workspace.
# Usage:
#   git clone https://github.com/saieswaramurali/arm_stack.git
#   cd arm_stack
#   ./setup.sh
set -euo pipefail

ROS_DISTRO_EXPECTED="humble"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log()  { echo -e "\n[setup] $*"; }
fail() { echo -e "\n[setup] ERROR: $*" >&2; exit 1; }

# 1. Sanity checks -----------------------------------------------------------
[ -d "$WS_DIR/src" ] || fail "run this from the repo root (src/ not found)"

if [ ! -f "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash" ]; then
    fail "ROS 2 ${ROS_DISTRO_EXPECTED} not found at /opt/ros/${ROS_DISTRO_EXPECTED}.
Install it first: https://docs.ros.org/en/humble/Installation.html"
fi

log "sourcing ROS 2 ${ROS_DISTRO_EXPECTED}"
# ROS setup.bash reads variables that may be unset; relax nounset around it
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
set -u

# 2. Base tooling ------------------------------------------------------------
log "installing base tooling (sudo required)"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    build-essential \
    git

# 3. rosdep ------------------------------------------------------------------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "initializing rosdep (first time on this machine)"
    sudo rosdep init
fi
log "updating rosdep index"
rosdep update

log "installing package dependencies via rosdep"
rosdep install --from-paths "$WS_DIR/src" --ignore-src -y --rosdistro "$ROS_DISTRO_EXPECTED"

# 4. Explicit runtime deps ---------------------------------------------------
# Covers anything rosdep may miss and the simulator/planning stack.
log "installing simulator and planning stack packages"
sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO_EXPECTED}-moveit" \
    "ros-${ROS_DISTRO_EXPECTED}-moveit-task-constructor-core" \
    "ros-${ROS_DISTRO_EXPECTED}-ros2-control" \
    "ros-${ROS_DISTRO_EXPECTED}-ros2-controllers" \
    "ros-${ROS_DISTRO_EXPECTED}-ros-gz" \
    "ros-${ROS_DISTRO_EXPECTED}-gz-ros2-control" \
    "ros-${ROS_DISTRO_EXPECTED}-xacro" \
    "ros-${ROS_DISTRO_EXPECTED}-cv-bridge" \
    liburdfdom-tools

# 5. Build -------------------------------------------------------------------
log "building the workspace (colcon)"
cd "$WS_DIR"
colcon build --symlink-install

# 6. Done --------------------------------------------------------------------
log "build complete. To use the workspace in every new terminal:"
echo "    source /opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
echo "    source ${WS_DIR}/install/setup.bash"
echo ""
echo "Optional: add both lines to ~/.bashrc"
echo ""
echo "Quick test:"
echo "    ros2 launch arm_bringup bringup.launch.py"
