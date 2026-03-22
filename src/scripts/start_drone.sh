#!/usr/bin/env bash
#
# Drone Ranger one-click launcher
# Start order: MicroXRCEAgent -> PX4 SITL -> ROS-TCP-Endpoint -> odom_bridge -> Mission
#
# Preflight "Attitude failure (roll)": EKF/IMU not ready yet, or tilt/innovation over limit.
#   - Prefer: increase MISSION_DELAY_SEC (e.g. 40) so auto_arm runs after EKF converges.
#   - PX4_PARAM_WAIT: delay before NAV_DLL_ACT + optional params (default 18s).
#   - Last resort (SITL only): PX4_POST_BOOT_CMDS to relax specific checks — see PX4 param docs (tmux mode only).
#
# Usage:
#   ./start_drone.sh
#   ./start_drone.sh --no-tmux
#   ./start_drone.sh --no-px4
#   ./start_drone.sh --mission offboard_takeoff
#   ./start_drone.sh --install-deps
#

set -e

# ============== Config (override via env vars) ==============
DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
DRONE_WS="${DRONE_WS:-$DRONE_ROOT/drone_ws}"
PX4_DIR="${PX4_DIR:-$DRONE_ROOT/PX4-Autopilot}"
# Host IP for Unity/QGC; if empty, auto-detect
ROS_IP="${ROS_IP:-}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
# odom_bridge.py: default to scripts/odom_bridge.py (repo-local)
if [[ -z "${ODOM_BRIDGE_SCRIPT}" ]]; then
  if [[ -f "$DRONE_ROOT/Drone-Ranger/odom_bridge.py" ]]; then
    ODOM_BRIDGE_SCRIPT="$DRONE_ROOT/Drone-Ranger/odom_bridge.py"
  fi
fi
# Default mission node name
DEFAULT_MISSION="${DEFAULT_MISSION:-offboard_takeoff}"
# PX4 SITL type: none (no sim) or gz_x500 etc.
PX4_SITL_TYPE="${PX4_SITL_TYPE:-none}"
# Logs directory (used with --no-tmux)
LOG_DIR="${LOG_DIR:-$DRONE_ROOT/logs}"
# How many seconds to wait before starting mission (time for Unity Play). 0 = start immediately
MISSION_DELAY_SEC="${MISSION_DELAY_SEC:-25}"
# After PX4 boots, wait this many seconds before sending NAV_DLL_ACT / optional params (EKF needs time)
PX4_PARAM_WAIT="${PX4_PARAM_WAIT:-18}"
# Optional: extra PX4 shell commands after boot (one command per line). Dev / SITL only — review each param.
# Example (unsafe on real aircraft): relax checks; prefer fixing sensors / waiting for EKF instead.
#   export PX4_POST_BOOT_CMDS=$'param set COM_ARM_IMU_ACC 3.0\nparam set COM_ARM_IMU_GYR 0.25'
PX4_POST_BOOT_CMDS="${PX4_POST_BOOT_CMDS:-}"

# ============== 解析参数 ==============
USE_TMUX=true
START_AGENT=true
START_PX4=true
START_TCP=true
START_ODOM_BRIDGE=true
INSTALL_DEPS=false
MISSION_DELAY=25
MISSION_DELAY_SET=false
MISSION="$DEFAULT_MISSION"
MISSION_PKG=""
QGC_IP=""
QGC_WAIT=30

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-tmux)       USE_TMUX=false; shift ;;
    --no-agent)      START_AGENT=false; shift ;;
    --no-px4)        START_PX4=false; shift ;;
    --no-tcp)        START_TCP=false; shift ;;
    --no-odom-bridge) START_ODOM_BRIDGE=false; shift ;;
    --qgc-ip)
      QGC_IP="$2"
      shift 2
      ;;
    --qgc-wait)
      QGC_WAIT="$2"
      shift 2
      ;;
    --mission-delay)
      MISSION_DELAY="$2"
      MISSION_DELAY_SET=true
      shift 2
      ;;
    --mission)
      MISSION="$2"
      shift 2
      ;;
    --install-deps)
      INSTALL_DEPS=true
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--no-tmux] [--no-agent] [--no-px4] [--no-tcp] [--no-odom-bridge] [--qgc-ip IP] [--qgc-wait SEC] [--mission-delay SEC] [--mission NAME]"
      echo "       $0 --install-deps"
      echo "  --qgc-ip IP      after PX4 boots, send mavlink for QGC at IP (e.g. 10.211.55.2)"
      echo "  --qgc-wait SEC   wait SEC seconds before sending mavlink (default 30)"
      echo "  --mission-delay  wait SEC seconds before starting mission (default 25, for Unity Play)"
      echo "  Env: MISSION_DELAY_SEC, PX4_PARAM_WAIT, PX4_POST_BOOT_CMDS (see script header)"
      echo "  --mission        offboard_takeoff, offboard_square_mission, orbit_point, haoran_unity_pursuit, ..."
      echo "  --install-deps   install missing apt dependencies (px4_msgs, tmux, etc.)"
      exit 0
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

# 仅当未传 --mission-delay 时用环境变量
[[ "$MISSION_DELAY_SET" != true && -n "$MISSION_DELAY_SEC" ]] && MISSION_DELAY="$MISSION_DELAY_SEC"

# 判定 mission 所在包
case "$MISSION" in
  offboard_takeoff|offboard_square_mission|offboard_orbit_mission|obstacle_avoidance_mission|obstacle_publisher)
    MISSION_PKG="offboard_test"
    ;;
  unity_house_avoidance)
    MISSION_PKG="offboard_avoidance_unity"
    ;;
  orbit_point|forward_mission|simple_takeoff|square_flight|circle_flight|figure_eight|waypoint_mission|house_inspection|pursuit_flight|rrt_pursuit_single|haoran_unity_pursuit)
    MISSION_PKG="drone_autonomy"
    ;;
  *)
    echo "Unknown mission: $MISSION (use --mission NAME)"
    exit 1
    ;;
esac

# 自动检测本机 IP（仅当 ROS_IP 未设置时）
get_ros_ip() {
  if [[ -n "$ROS_IP" ]]; then
    echo "$ROS_IP"
    return
  fi
  local ip
  ip=$(ip -4 route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[\d.]+' || true)
  if [[ -z "$ip" ]]; then
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  fi
  echo "${ip:-127.0.0.1}"
}

ROS_IP="$(get_ros_ip)"
echo "[start_drone] ROS_IP=$ROS_IP ROS_TCP_PORT=$ROS_TCP_PORT Mission=$MISSION_PKG/$MISSION MissionDelay=${MISSION_DELAY}s"

# PX4 versions vary; this repo supports "none" and Gazebo ("gz_*") targets.
# If user passes an unsupported target (e.g. jmavsim), fall back to "none".
if [[ "$PX4_SITL_TYPE" == "jmavsim" ]]; then
  echo "[start_drone] Warning: PX4_SITL_TYPE=jmavsim is not supported by this PX4 build. Falling back to PX4_SITL_TYPE=none."
  PX4_SITL_TYPE="none"
fi

# --------------- 清理旧进程，保证再次执行与第一次一致 ---------------
tmux kill-session -t drone 2>/dev/null || true
LOG_DIR="${LOG_DIR:-$DRONE_ROOT/logs}"
if [[ -d "$LOG_DIR" ]]; then
  for f in "$LOG_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    pid=$(cat "$f" 2>/dev/null)
    if [[ -n "$pid" ]] && [[ -d /proc/$pid ]]; then
      kill "$pid" 2>/dev/null || true
      echo "[start_drone] Killed previous process (PID $pid from $(basename "$f"))"
    fi
    rm -f "$f"
  done
fi

# 释放 TCP endpoint 端口并等待真正空闲，避免 "Address already in use"
free_port() {
  local port=$1
  local max_wait=5
  if command -v fuser &>/dev/null; then
    fuser -k "$port/tcp" 2>/dev/null || true
  else
    local pid
    pid=$(lsof -i ":$port" -t 2>/dev/null | head -1)
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  fi
  while [[ $max_wait -gt 0 ]]; do
    if ! ss -tlnp 2>/dev/null | grep -q ":$port "; then
      [[ $max_wait -lt 5 ]] && echo "[start_drone] Port $port is free"
      return 0
    fi
    sleep 1
    max_wait=$((max_wait - 1))
  done
  echo "[start_drone] Warning: port $port may still be in use"
}
if [[ "$START_TCP" == true ]]; then
  free_port "$ROS_TCP_PORT"
fi

# 检查必要路径
if [[ ! -d "$DRONE_WS" ]]; then
  echo "Error: DRONE_WS not found: $DRONE_WS"
  exit 1
fi
if [[ "$START_PX4" == true && ! -d "$PX4_DIR" ]]; then
  echo "Error: PX4_DIR not found: $PX4_DIR"
  exit 1
fi
if [[ "$START_ODOM_BRIDGE" == true && -n "$ODOM_BRIDGE_SCRIPT" && ! -f "$ODOM_BRIDGE_SCRIPT" ]]; then
  echo "Warning: ODOM_BRIDGE_SCRIPT not found: $ODOM_BRIDGE_SCRIPT (skipping odom_bridge)"
  START_ODOM_BRIDGE=false
fi
if [[ -z "${ODOM_BRIDGE_SCRIPT:-}" ]]; then
  START_ODOM_BRIDGE=false
fi

if [[ "$INSTALL_DEPS" == true ]]; then
  if [[ -x "$DRONE_ROOT/scripts/install_deps.sh" ]]; then
    "$DRONE_ROOT/scripts/install_deps.sh"
  else
    echo "Error: install_deps.sh not found or not executable at $DRONE_ROOT/scripts/install_deps.sh"
    exit 1
  fi
fi

source /opt/ros/humble/setup.bash
if [[ -f "$DRONE_WS/install/setup.bash" ]]; then
  source "$DRONE_WS/install/setup.bash"
else
  echo "Error: $DRONE_WS/install/setup.bash not found. Build the workspace first:"
  echo "  cd $DRONE_WS && colcon build --symlink-install"
  exit 1
fi

# --------------- Mission executable check ---------------
# Ensure the selected mission node is installed in this workspace.
# For Python/ament console_scripts, the executable is typically located under:
#   $DRONE_WS/install/<pkg>/lib/<pkg>/<console_script_name>
is_mission_executable_installed() {
  local pkg="$1"
  local exe="$2"

  local candidates=(
    "$DRONE_WS/install/$pkg/lib/$pkg/$exe"
    "$DRONE_WS/install/lib/$pkg/$exe"
    "$DRONE_WS/install/bin/$exe"
    "$DRONE_WS/install/$pkg/bin/$exe"
  )

  local c
  for c in "${candidates[@]}"; do
    if [[ -x "$c" ]]; then
      return 0
    fi
  done
  return 1
}

if [[ -n "$MISSION_PKG" && -n "$MISSION" ]]; then
  if ! is_mission_executable_installed "$MISSION_PKG" "$MISSION"; then
    echo "[start_drone] Mission executable not found in workspace: $MISSION_PKG/$MISSION"

    if [[ ! -x "$DRONE_ROOT/scripts/build_ros.sh" ]]; then
      echo "Error: build_ros.sh not found or not executable at $DRONE_ROOT/scripts/build_ros.sh"
      exit 1
    fi

    # If PX4 message packages are missing from the workspace source tree,
    # build_ros.sh can clone them (via --with-px4) so mission packages compile.
    build_args=()
    if [[ ! -d "$DRONE_WS/src/px4_msgs" || ! -d "$DRONE_WS/src/px4_ros_com" ]]; then
      build_args+=("--with-px4")
    fi
    build_args+=("$MISSION_PKG")

    echo "[start_drone] Building mission package: $MISSION_PKG"
    "$DRONE_ROOT/scripts/build_ros.sh" "${build_args[@]}"

    # Reload the workspace environment after building.
    source "$DRONE_WS/install/setup.bash"

    if ! is_mission_executable_installed "$MISSION_PKG" "$MISSION"; then
      echo "Error: Mission still not available after build: $MISSION_PKG/$MISSION"
      exit 1
    fi
  fi
fi

if ! ros2 pkg list 2>/dev/null | grep -q "^px4_msgs$"; then
  echo "[start_drone] Warning: px4_msgs is NOT installed in this ROS environment."
  echo "  Fix: sudo apt install ros-humble-px4-msgs"
  echo "  (Or run: $0 --install-deps)"
fi

# --------------- 使用 tmux 启动 ---------------
run_tmux() {
  local session="drone"
  tmux kill-session -t "$session" 2>/dev/null || true
  tmux new-session -d -s "$session" -n "agent"

  if [[ "$START_AGENT" == true ]]; then
    tmux send-keys -t "$session:agent" "MicroXRCEAgent udp4 -p 8888" Enter
  else
    tmux send-keys -t "$session:agent" "echo 'MicroXRCEAgent skipped (--no-agent)'" Enter
  fi
  tmux new-window -t "$session" -n "px4"

  if [[ "$START_PX4" == true ]]; then
    if [[ "$PX4_SITL_TYPE" == "none" ]]; then
      tmux send-keys -t "$session:px4" "cd $PX4_DIR && make px4_sitl_default none" Enter
    else
      tmux send-keys -t "$session:px4" "cd $PX4_DIR && make px4_sitl_default $PX4_SITL_TYPE" Enter
    fi
  else
    tmux send-keys -t "$session:px4" "echo 'PX4 skipped (--no-px4)'" Enter
  fi
  tmux new-window -t "$session" -n "tcp"

  if [[ "$START_TCP" == true ]]; then
    tmux send-keys -t "$session:tcp" "source /opt/ros/humble/setup.bash && source $DRONE_WS/install/setup.bash" Enter
    tmux send-keys -t "$session:tcp" "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=$ROS_IP -p ROS_TCP_PORT:=$ROS_TCP_PORT" Enter
  else
    tmux send-keys -t "$session:tcp" "echo 'ROS-TCP-Endpoint skipped (--no-tcp)'" Enter
  fi
  tmux new-window -t "$session" -n "ros"

  if [[ "$START_ODOM_BRIDGE" == true ]]; then
    tmux send-keys -t "$session:ros" "source /opt/ros/humble/setup.bash && source $DRONE_WS/install/setup.bash && python3 $ODOM_BRIDGE_SCRIPT" Enter
  else
    tmux send-keys -t "$session:ros" "echo 'odom_bridge skipped (set ODOM_BRIDGE_SCRIPT to enable)'" Enter
  fi

  tmux new-window -t "$session" -n "mission"

  tmux send-keys -t "$session:mission" "source /opt/ros/humble/setup.bash && source $DRONE_WS/install/setup.bash" Enter
  if [[ "$MISSION_DELAY" -gt 0 ]]; then
    tmux send-keys -t "$session:mission" "echo \"Waiting ${MISSION_DELAY}s: PX4/param set, then start mission. Please Unity Play before this ends.\"; sleep $MISSION_DELAY && ros2 run $MISSION_PKG $MISSION" Enter
  else
    tmux send-keys -t "$session:mission" "ros2 run $MISSION_PKG $MISSION" Enter
  fi

  echo "[start_drone] Tmux session '$session' started. Attach with: tmux attach -t $session"
  echo "  Windows: agent -> px4 -> tcp -> ros -> mission"
}

# --------------- 后台进程 + 日志 ---------------
run_background() {
  mkdir -p "$LOG_DIR"
  echo "[start_drone] Logs in $LOG_DIR"

  if [[ "$START_AGENT" == true ]]; then
    MicroXRCEAgent udp4 -p 8888 &> "$LOG_DIR/agent.log" &
    echo $! > "$LOG_DIR/agent.pid"
    sleep 1
  fi

  if [[ "$START_PX4" == true ]]; then
    (cd "$PX4_DIR" && make px4_sitl_default "$PX4_SITL_TYPE") &> "$LOG_DIR/px4.log" &
    echo $! > "$LOG_DIR/px4.pid"
    echo "[start_drone] PX4 starting in background (wait ~20s before mission)"
    sleep 3
  fi

  if [[ "$START_TCP" == true ]]; then
    (
      source /opt/ros/humble/setup.bash
      source "$DRONE_WS/install/setup.bash"
      ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$ROS_IP" -p ROS_TCP_PORT:="$ROS_TCP_PORT"
    ) &> "$LOG_DIR/tcp.log" &
    echo $! > "$LOG_DIR/tcp.pid"
    sleep 1
  fi

  if [[ "$START_ODOM_BRIDGE" == true ]]; then
    (
      source /opt/ros/humble/setup.bash
      source "$DRONE_WS/install/setup.bash"
      python3 "$ODOM_BRIDGE_SCRIPT"
    ) &> "$LOG_DIR/odom_bridge.log" &
    echo $! > "$LOG_DIR/odom_bridge.pid"
    sleep 1
  fi

  (
    source /opt/ros/humble/setup.bash
    source "$DRONE_WS/install/setup.bash"
    if [[ "$MISSION_DELAY" -gt 0 ]]; then
      echo "[start_drone] Waiting ${MISSION_DELAY}s (PX4/param ready, Unity Play), then starting mission..."
      sleep "$MISSION_DELAY"
    fi
    ros2 run "$MISSION_PKG" "$MISSION"
  ) &> "$LOG_DIR/mission.log" &
  echo $! > "$LOG_DIR/mission.pid"
  echo "[start_drone] Mission $MISSION_PKG/$MISSION running (delay ${MISSION_DELAY}s). PID in $LOG_DIR/*.pid"
}

# --------------- 执行 ---------------
# PX4 就绪后自动设 NAV_DLL_ACT 0，避免 GCS 预检挡解锁（在 mission 前完成）
if [[ "$USE_TMUX" == true ]]; then
  run_tmux
  if [[ "$START_PX4" == true ]]; then
    (
      sleep "$PX4_PARAM_WAIT"
      tmux send-keys -t drone:px4 "param set NAV_DLL_ACT 0" Enter
      sleep 0.5
      if [[ -n "$PX4_POST_BOOT_CMDS" ]]; then
        echo "[start_drone] Running PX4_POST_BOOT_CMDS (${#PX4_POST_BOOT_CMDS} chars)..."
        while IFS= read -r _line || [[ -n "$_line" ]]; do
          [[ -z "${_line//[$'\t\r\n ']}" ]] && continue
          tmux send-keys -t drone:px4 "$_line" Enter
          sleep 0.35
        done <<< "$PX4_POST_BOOT_CMDS"
      fi
    ) &
    echo "[start_drone] PX4 param NAV_DLL_ACT=0 will be set in ${PX4_PARAM_WAIT}s (no GCS required to arm)."
    if [[ -n "$PX4_POST_BOOT_CMDS" ]]; then
      echo "[start_drone] PX4_POST_BOOT_CMDS will run after that (dev only)."
    fi

    # Ensure uXRCE-DDS client is running so /fmu/out/* topics exist in ROS 2.
    # Some PX4 SITL configs do not auto-start it.
    (
      sleep "$PX4_PARAM_WAIT"
      tmux send-keys -t drone:px4 "uxrce_dds_client status" Enter
      sleep 1
      # Explicitly set host to localhost; improves reliability across SITL variants.
      tmux send-keys -t drone:px4 "uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888" Enter
      sleep 1
      tmux send-keys -t drone:px4 "uxrce_dds_client status" Enter
    ) &
    echo "[start_drone] Will ensure uxrce_dds_client is running (UDP 8888) after PX4 boots."
  fi
  if [[ -n "$QGC_IP" && "$START_PX4" == true ]]; then
    (
      sleep "$QGC_WAIT"
      tmux send-keys -t drone:px4 "mavlink stop-all" Enter
      sleep 2
      tmux send-keys -t drone:px4 "mavlink start -x -u 14550 -r 40000 -t $QGC_IP" Enter
    ) &
    echo "[start_drone] QGC mavlink will be sent to PX4 in ${QGC_WAIT}s for $QGC_IP (add UDP 14550 in QGC)."
  fi
else
  run_background
fi
