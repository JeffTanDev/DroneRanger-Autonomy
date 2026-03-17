#!/usr/bin/env bash
# Install/build dependencies for Drone Ranger (ROS 2 Humble + PX4 + Unity bridge).
#
# Usage:
#   ./install_deps.sh
#   ./install_deps.sh --skip-px4
#   ./install_deps.sh --skip-microxrce
#
# Notes:
# - This script installs system packages via apt.
# - It does NOT configure Unity; it only installs ROS/PX4 side dependencies.

set -euo pipefail

DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
DRONE_WS="${DRONE_WS:-$DRONE_ROOT/drone_ws}"

SKIP_PX4=false
SKIP_MICROXRCE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-px4) SKIP_PX4=true; shift ;;
    --skip-microxrce) SKIP_MICROXRCE=true; shift ;;
    --help|-h)
      echo "Usage: $0 [--skip-px4] [--skip-microxrce]"
      exit 0
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

echo "[install_deps] DRONE_ROOT=$DRONE_ROOT"

sudo apt update

# Base utilities used by scripts
sudo apt install -y \
  tmux \
  psmisc \
  lsof \
  git \
  curl \
  ca-certificates \
  python3 \
  python3-pip

# ROS 2 build tools (safe even if already installed)
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep

# PX4 message definitions (required for ros2 topic echo /fmu/out/* and for our nodes importing px4_msgs)
sudo apt install -y ros-humble-px4-msgs

if [[ "$SKIP_MICROXRCE" != true ]]; then
  # Micro XRCE DDS Agent (required when using uXRCE-DDS with PX4 SITL)
  # Package name varies by distro; try apt first.
  if apt-cache show micro-xrce-dds-agent &>/dev/null; then
    sudo apt install -y micro-xrce-dds-agent
  else
    echo "[install_deps] micro-xrce-dds-agent not found in apt. Build from source if needed:"
    echo "  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
    echo "  cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build"
    echo "  cmake .. && make -j\$(nproc) && sudo make install && sudo ldconfig"
  fi
fi

if [[ "$SKIP_PX4" != true ]]; then
  # PX4 toolchain deps (for building PX4 SITL)
  if [[ -d "$DRONE_ROOT/PX4-Autopilot" ]]; then
    echo "[install_deps] PX4-Autopilot detected. Installing PX4 build dependencies..."
    (cd "$DRONE_ROOT/PX4-Autopilot" && bash ./Tools/setup/ubuntu.sh)
  else
    echo "[install_deps] PX4-Autopilot not found at $DRONE_ROOT/PX4-Autopilot (skipping PX4 deps)."
    echo "  If you want PX4 SITL: git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
  fi
fi

echo "[install_deps] Done."
echo "Next:"
echo "  source /opt/ros/humble/setup.bash"
echo "  # build your workspace if needed:"
echo "  cd \"$DRONE_WS\" && rosdep update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
