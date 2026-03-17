#!/usr/bin/env bash
# Stop processes started by start_drone.sh, so re-running is clean and repeatable.

DRONE_ROOT="${DRONE_ROOT:-/home/parallels/drone_local}"
LOG_DIR="${LOG_DIR:-$DRONE_ROOT/logs}"

echo "[stop_drone] Stopping drone session..."

# Kill tmux session (kills processes in windows)
tmux kill-session -t drone 2>/dev/null && echo "[stop_drone] Killed tmux session 'drone'" || true

# Force-free common ports used by this project (best effort)
free_port() {
  local port=$1
  if command -v fuser &>/dev/null; then
    fuser -k "$port/tcp" 2>/dev/null || true
  else
    # fall back to lsof if available
    if command -v lsof &>/dev/null; then
      local pids
      pids=$(lsof -i "tcp:$port" -t 2>/dev/null || true)
      if [[ -n "$pids" ]]; then
        kill $pids 2>/dev/null || true
      fi
    fi
  fi
}

# ROS-TCP-Endpoint default port
free_port "${ROS_TCP_PORT:-10000}"

# Kill background processes recorded in *.pid files
if [[ -d "$LOG_DIR" ]]; then
  for f in "$LOG_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    pid=$(cat "$f" 2>/dev/null)
    if [[ -n "$pid" ]]; then
      if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null && echo "[stop_drone] Killed PID $pid ($(basename "$f" .pid))"
      fi
      rm -f "$f"
    fi
  done
fi

# Optional: kill potential leftovers by name (use carefully)
pkill -f "ros2 run ros_tcp_endpoint default_server_endpoint" 2>/dev/null || true
pkill -f "$DRONE_ROOT/scripts/odom_bridge.py" 2>/dev/null || true
pkill -f "$DRONE_ROOT/Drone-Ranger/odom_bridge.py" 2>/dev/null || true
pkill -f "python3 .*odom_bridge.py" 2>/dev/null || true
pkill -f "ros2 run offboard_test offboard_" 2>/dev/null || true
pkill -f "ros2 run offboard_test offboard_takeoff" 2>/dev/null || true
pkill -f "ros2 run offboard_test offboard_orbit" 2>/dev/null || true
# In our workspace, missions are often launched via installed entrypoints (no `ros2 run` in cmdline).
pkill -f "/home/parallels/drone_local/drone_ws/install/offboard_test/lib/offboard_test/offboard_" 2>/dev/null || true
pkill -f "/home/parallels/drone_local/drone_ws/install/offboard_test/lib/offboard_test/offboard_takeoff" 2>/dev/null || true
pkill -f MicroXRCEAgent 2>/dev/null || true

# Clear ROS graph cache so `ros2 node list` reflects reality immediately.
if command -v ros2 &>/dev/null; then
  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start 2>/dev/null || true
fi

echo "[stop_drone] Done. You can run ./start_drone.sh again."
