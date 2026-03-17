#!/usr/bin/env bash
# One-click build/update for drone_ws (ROS 2).
# Usage: ./build_ros.sh [packages...]
# - No args: build full workspace
# - With args: build selected packages, e.g.:
#     ./build_ros.sh offboard_test drone_autonomy
#
# Optional flags:
#   --with-px4   Ensure px4_msgs + px4_ros_com exist under drone_ws/src (clone if missing)

set -e
DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
DRONE_WS="${DRONE_WS:-$DRONE_ROOT/drone_ws}"
WITH_PX4=false

if [[ ! -d "$DRONE_WS" ]]; then
  echo "Error: DRONE_WS not found: $DRONE_WS"
  exit 1
fi

source /opt/ros/humble/setup.bash
cd "$DRONE_WS"

if [[ $# -gt 0 && "$1" == "--with-px4" ]]; then
  WITH_PX4=true
  shift
fi

if [[ "$WITH_PX4" == true ]]; then
  mkdir -p "$DRONE_WS/src"
  if [[ ! -d "$DRONE_WS/src/px4_msgs" ]]; then
    echo "[build_ros] Cloning px4_msgs..."
    git clone https://github.com/PX4/px4_msgs.git "$DRONE_WS/src/px4_msgs"
  fi
  if [[ ! -d "$DRONE_WS/src/px4_ros_com" ]]; then
    echo "[build_ros] Cloning px4_ros_com..."
    git clone https://github.com/PX4/px4_ros_com.git "$DRONE_WS/src/px4_ros_com"
  fi

  if command -v rosdep &>/dev/null; then
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y
  else
    echo "[build_ros] Warning: rosdep not found; skipping dependency install."
  fi
fi

if [[ $# -gt 0 ]]; then
  echo "[build_ros] Building packages: $*"
  colcon build --symlink-install --packages-select "$@"
else
  echo "[build_ros] Building full workspace (drone_ws)"
  colcon build --symlink-install
fi

echo ""
echo "[build_ros] Done. To use: source $DRONE_WS/install/setup.bash"
