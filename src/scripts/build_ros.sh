#!/usr/bin/env bash
# One-click build/update for drone_ws (ROS 2).
#
# This script supports using git submodules for official dependencies:
#   - ROS-TCP-Endpoint (Unity-Technologies)
#   - px4_msgs (PX4)
#   - px4_ros_com (PX4)
#
# Usage: ./build_ros.sh [--with-px4] [--skip-submodules] [--no-rosdep] [packages...]
# - No packages: build full workspace
# - With packages: build packages plus their dependency chain
#
# Optional flags:
#   --with-px4         Ensure px4_msgs + px4_ros_com exist under drone_ws/src (fallback clone if needed)
#   --skip-submodules  Skip `git submodule update --init --recursive` even if .gitmodules exists
#   --no-rosdep        Skip rosdep install step

set -euo pipefail

DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
DRONE_WS="${DRONE_WS:-$DRONE_ROOT/drone_ws}"

WITH_PX4=false
SKIP_SUBMODULES=false
NO_ROSDEP=false

if [[ ! -d "$DRONE_WS" ]]; then
  echo "Error: DRONE_WS not found: $DRONE_WS"
  exit 1
fi

# ROS setup scripts may reference optional/unbound variables.
# Temporarily disable `-u` while sourcing to avoid "unbound variable" crashes.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$DRONE_WS"

PACKAGES=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-px4)        WITH_PX4=true; shift ;;
    --skip-submodules) SKIP_SUBMODULES=true; shift ;;
    --no-rosdep)       NO_ROSDEP=true; shift ;;
    --help|-h)
      echo "Usage: $0 [--with-px4] [--skip-submodules] [--no-rosdep] [packages...]"
      exit 0
      ;;
    *)
      PACKAGES+=("$1")
      shift
      ;;
  esac
done

ensure_official_deps_via_submodules() {
  if [[ "$SKIP_SUBMODULES" == true ]]; then
    return 0
  fi
  if [[ -f "$DRONE_WS/.gitmodules" ]]; then
    echo "[build_ros] Updating git submodules (official deps)..."
    git submodule update --init --recursive
  else
    echo "[build_ros] No .gitmodules found at $DRONE_WS; skipping submodule update."
  fi
}

fallback_clone_if_missing() {
  mkdir -p "$DRONE_WS/src"

  # ROS-TCP-Endpoint
  if [[ ! -d "$DRONE_WS/src/ROS-TCP-Endpoint" ]]; then
    echo "[build_ros] ROS-TCP-Endpoint missing; cloning official repo (fallback)..."
    git clone --recursive -b main-ros2 \
      https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
      "$DRONE_WS/src/ROS-TCP-Endpoint" || true
  fi

  # PX4 ROS 2 packages
  if [[ "$WITH_PX4" == true ]]; then
    if [[ ! -d "$DRONE_WS/src/px4_msgs" ]]; then
      echo "[build_ros] px4_msgs missing; cloning official repo (fallback)..."
      git clone --recursive -b main \
        https://github.com/PX4/px4_msgs.git \
        "$DRONE_WS/src/px4_msgs" || true
    fi
    if [[ ! -d "$DRONE_WS/src/px4_ros_com" ]]; then
      echo "[build_ros] px4_ros_com missing; cloning official repo (fallback)..."
      git clone --recursive -b main \
        https://github.com/PX4/px4_ros_com.git \
        "$DRONE_WS/src/px4_ros_com" || true
    fi
  fi
}

ensure_deps_sources_present() {
  ensure_official_deps_via_submodules
  # If submodule wasn't set up yet, we can still fall back to cloning.
  fallback_clone_if_missing
}

ensure_deps_sources_present

if [[ "$NO_ROSDEP" != true ]]; then
  if command -v rosdep &>/dev/null; then
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y
  else
    echo "[build_ros] Warning: rosdep not found; skipping dependency install."
  fi
fi

if [[ ${#PACKAGES[@]} -gt 0 ]]; then
  echo "[build_ros] Building packages up-to (with deps): ${PACKAGES[*]}"
  colcon build --symlink-install --packages-up-to "${PACKAGES[@]}"
else
  echo "[build_ros] Building full workspace (drone_ws)"
  colcon build --symlink-install
fi

echo ""
echo "[build_ros] Done. To use: source $DRONE_WS/install/setup.bash"
