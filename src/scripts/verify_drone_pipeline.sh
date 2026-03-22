#!/usr/bin/env bash
#
# (If needed) run with: `bash /home/parallels/drone_local/scripts/verify_drone_pipeline.sh`
#
# Drone Ranger communication verification helper.
#
# What it guarantees (best-effort):
# - ROS 2 + ros_tcp_endpoint server is running (TCP port listening locally)
# - uXRCE-DDS agent exists (UDP 8888)
# - odom_bridge is running and /drone/odom shows up
# - After you press Unity "Play", Unity publishers should appear:
#   - /drone/gps
#   - /static_cam/target_gps
#
# Then it prints one sample message from each GPS topic so you can verify
# the coordinate pipeline quickly.
#
# Usage:
#   ./verify_drone_pipeline.sh
#   ./verify_drone_pipeline.sh --no-tmux
#   ./verify_drone_pipeline.sh --no-px4
#   ./verify_drone_pipeline.sh --unity-wait 40
#
set -euo pipefail

# Ensure bash script is executable bit is not guaranteed in all environments.
# You can run it with: `bash scripts/verify_drone_pipeline.sh ...`

DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
DRONE_WS="${DRONE_WS:-$DRONE_ROOT/drone_ws}"
PX4_DIR="${PX4_DIR:-$DRONE_ROOT/PX4-Autopilot}"

ROS_IP="${ROS_IP:-}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"

# odom_bridge.py: default to repo-local
if [[ -z "${ODOM_BRIDGE_SCRIPT:-}" ]]; then
  if [[ -f "$DRONE_ROOT/Drone-Ranger/odom_bridge.py" ]]; then
    ODOM_BRIDGE_SCRIPT="$DRONE_ROOT/Drone-Ranger/odom_bridge.py"
  fi
fi

USE_TMUX=true
START_AGENT=true
START_PX4=true
START_TCP=true
START_ODOM_BRIDGE=true

UNITY_WAIT_SEC="${UNITY_WAIT_SEC:-25}"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-30}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-tmux) USE_TMUX=false; shift ;;
    --no-agent) START_AGENT=false; shift ;;
    --no-px4) START_PX4=false; shift ;;
    --no-tcp) START_TCP=false; shift ;;
    --no-odom-bridge) START_ODOM_BRIDGE=false; shift ;;
    --unity-wait) UNITY_WAIT_SEC="$2"; shift 2 ;;
    --topic-timeout) TOPIC_TIMEOUT_SEC="$2"; shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--no-tmux] [--no-agent] [--no-px4] [--no-tcp] [--no-odom-bridge] [--unity-wait SEC] [--topic-timeout SEC]"
      exit 0
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

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
LOG_DIR="${LOG_DIR:-$DRONE_ROOT/logs}"
mkdir -p "$LOG_DIR"

cleanup_old_pids() {
  if [[ -d "$LOG_DIR" ]]; then
    for f in "$LOG_DIR"/*.pid; do
      [[ -f "$f" ]] || continue
      # Only kill pid files created by this verification script (prefix match).
      if [[ "$(basename "$f")" != verify_*.pid ]]; then
        continue
      fi
      pid="$(cat "$f" 2>/dev/null || true)"
      if [[ -n "$pid" && -d /proc/"$pid" ]]; then
        kill "$pid" 2>/dev/null || true
      fi
      rm -f "$f"
    done
  fi
}

free_port() {
  local port=$1
  local max_wait=5
  if command -v fuser &>/dev/null; then
    fuser -k "$port/tcp" 2>/dev/null || true
  fi
  while [[ $max_wait -gt 0 ]]; do
    if ! ss -tlnp 2>/dev/null | grep -q ":$port "; then
      return 0
    fi
    sleep 1
    max_wait=$((max_wait - 1))
  done
  return 0
}

if [[ "$START_TCP" == true ]]; then
  free_port "$ROS_TCP_PORT"
fi

if [[ "$START_PX4" == true && ! -d "$PX4_DIR" ]]; then
  echo "Error: PX4_DIR not found: $PX4_DIR"
  exit 1
fi
if [[ "$START_ODOM_BRIDGE" == true ]]; then
  if [[ ! -f "$ODOM_BRIDGE_SCRIPT" ]]; then
    echo "Warning: ODOM_BRIDGE_SCRIPT not found: $ODOM_BRIDGE_SCRIPT"
    echo "         disabling odom_bridge."
    START_ODOM_BRIDGE=false
  fi
fi

cleanup_old_pids

# Under `set -u`, sourcing ROS setup scripts can fail if they reference
# uninitialized variables (e.g. AMENT_TRACE_SETUP_FILES).
# Temporarily disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$DRONE_WS/install/setup.bash" ]]; then
  source "$DRONE_WS/install/setup.bash"
fi
set -u

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local t0
  t0="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    local now
    now="$(date +%s)"
    if [[ $((now - t0)) -ge $timeout_sec ]]; then
      return 1
    fi
    sleep 1
  done
}

sample_topic_once() {
  local topic="$1"
  echo
  echo "==== Sample: $topic (one message) ===="
  # `ros2 topic echo` in this ROS2 version supports `--once` (not `-n`).
  # timeout prevents blocking if Unity hasn't published yet.
  timeout 6 ros2 topic echo --once "$topic" || true
  echo "==== End: $topic ===="
}

run_tmux() {
  local session="drone_verify"
  tmux kill-session -t "$session" 2>/dev/null || true
  tmux new-session -d -s "$session" -n "agent"

  if [[ "$START_AGENT" == true ]]; then
    tmux send-keys -t "$session:agent" "MicroXRCEAgent udp4 -p 8888" Enter
  else
    tmux send-keys -t "$session:agent" "echo 'MicroXRCEAgent skipped (--no-agent)'" Enter
  fi

  tmux new-window -t "$session" -n "px4"
  if [[ "$START_PX4" == true ]]; then
    # If PX4 SITL type is provided via env, reuse it; default to none.
    local SITL="${PX4_SITL_TYPE:-none}"
    if [[ "$SITL" == "jmavsim" ]]; then
      SITL="none"
    fi
    tmux send-keys -t "$session:px4" "cd '$PX4_DIR' && make px4_sitl_default '$SITL'" Enter
  else
    tmux send-keys -t "$session:px4" "echo 'PX4 skipped (--no-px4)'" Enter
  fi

  tmux new-window -t "$session" -n "tcp"
  if [[ "$START_TCP" == true ]]; then
    tmux send-keys -t "$session:tcp" "source /opt/ros/humble/setup.bash && source '$DRONE_WS/install/setup.bash' && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=$ROS_IP -p ROS_TCP_PORT:=$ROS_TCP_PORT" Enter
  else
    tmux send-keys -t "$session:tcp" "echo 'ros_tcp_endpoint skipped (--no-tcp)'" Enter
  fi

  tmux new-window -t "$session" -n "ros"
  if [[ "$START_ODOM_BRIDGE" == true ]]; then
    tmux send-keys -t "$session:ros" "source /opt/ros/humble/setup.bash && source '$DRONE_WS/install/setup.bash' && python3 '$ODOM_BRIDGE_SCRIPT'" Enter
  else
    tmux send-keys -t "$session:ros" "echo 'odom_bridge skipped (--no-odom-bridge)'" Enter
  fi

  echo "[verify] tmux session '$session' started."
  echo "         windows: agent | px4 | tcp | ros"
  echo "         attach: tmux attach -t $session"

  echo "[verify] Waiting ${UNITY_WAIT_SEC}s for you to press Unity 'Play'..."
  sleep "$UNITY_WAIT_SEC"
}

run_background() {
  if [[ "$START_AGENT" == true ]]; then
    MicroXRCEAgent udp4 -p 8888 &> "$LOG_DIR/verify_agent.log" &
    echo $! > "$LOG_DIR/verify_agent.pid"
    sleep 1
  fi

  if [[ "$START_PX4" == true ]]; then
    (
      local SITL="${PX4_SITL_TYPE:-none}"
      if [[ "$SITL" == "jmavsim" ]]; then
        SITL="none"
      fi
      cd "$PX4_DIR"
      make px4_sitl_default "$SITL"
    ) &> "$LOG_DIR/verify_px4.log" &
    echo $! > "$LOG_DIR/verify_px4.pid"
    sleep 3
  fi

  if [[ "$START_TCP" == true ]]; then
    free_port "$ROS_TCP_PORT"
    (
      set +u
      source /opt/ros/humble/setup.bash
      if [[ -f "$DRONE_WS/install/setup.bash" ]]; then
        source "$DRONE_WS/install/setup.bash"
      fi
      set -u
      ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$ROS_IP" -p ROS_TCP_PORT:="$ROS_TCP_PORT"
    ) &> "$LOG_DIR/verify_tcp.log" &
    echo $! > "$LOG_DIR/verify_tcp.pid"
    sleep 2
  fi

  if [[ "$START_ODOM_BRIDGE" == true ]]; then
    (
      set +u
      source /opt/ros/humble/setup.bash
      if [[ -f "$DRONE_WS/install/setup.bash" ]]; then
        source "$DRONE_WS/install/setup.bash"
      fi
      set -u
      python3 "$ODOM_BRIDGE_SCRIPT"
    ) &> "$LOG_DIR/verify_odom_bridge.log" &
    echo $! > "$LOG_DIR/verify_odom_bridge.pid"
    sleep 1
  fi

  echo "[verify] Waiting ${UNITY_WAIT_SEC}s for you to press Unity 'Play'..."
  sleep "$UNITY_WAIT_SEC"
}

if [[ "$USE_TMUX" == true ]]; then
  run_tmux
else
  run_background
fi

echo
echo "[verify] Checking topics..."

if ! wait_for_topic "/drone/odom" "$TOPIC_TIMEOUT_SEC"; then
  echo "[verify][WARN] Topic /drone/odom not found within ${TOPIC_TIMEOUT_SEC}s."
  echo "                Check odom_bridge logs: $LOG_DIR/verify_odom_bridge.log (if using background) or tmux window 'ros' (if tmux)."
fi

if ! wait_for_topic "/drone/gps" "$TOPIC_TIMEOUT_SEC"; then
  echo "[verify][WARN] Topic /drone/gps not found within ${TOPIC_TIMEOUT_SEC}s."
  echo "                This usually means Unity 'Play' hasn't published yet or your DroneGpsPublisher isn't active."
fi

if ! wait_for_topic "/static_cam/target_gps" "$TOPIC_TIMEOUT_SEC"; then
  echo "[verify][WARN] Topic /static_cam/target_gps not found within ${TOPIC_TIMEOUT_SEC}s."
  echo "                This usually means Unity 'Play' hasn't published yet or StaticCamTargetGPSPublisher isn't active."
fi

# Print TCP port status as an extra communication sanity check.
if [[ "$START_TCP" == true ]]; then
  echo
  echo "[verify] TCP listener check:"
  ss -tlnp 2>/dev/null | grep -F ":$ROS_TCP_PORT " || echo "[verify][WARN] Port $ROS_TCP_PORT not listening locally."
fi

sample_topic_once "/drone/gps"
sample_topic_once "/static_cam/target_gps"

echo
echo "[verify] Done. If samples look plausible, you can proceed with your GPS/home/target verification."

