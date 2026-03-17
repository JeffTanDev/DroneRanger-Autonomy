#!/usr/bin/env bash
# Start ROS TCP Endpoint (Unity <-> ROS 2) and auto-free the port to avoid
# "Address already in use".
# Usage:
#   ./run_tcp_endpoint.sh
#   ./run_tcp_endpoint.sh [ROS_IP] [PORT]

set -e
DRONE_WS="${DRONE_WS:-/home/parallels/drone_local/drone_ws}"
ROS_IP="${1:-10.211.55.3}"
ROS_TCP_PORT="${2:-10000}"

# 释放端口
if command -v fuser &>/dev/null; then
  fuser -k "$ROS_TCP_PORT/tcp" 2>/dev/null && echo "[run_tcp_endpoint] Freed port $ROS_TCP_PORT" || true
else
  pid=$(lsof -i ":$ROS_TCP_PORT" -t 2>/dev/null | head -1)
  [[ -n "$pid" ]] && kill "$pid" 2>/dev/null && echo "[run_tcp_endpoint] Killed PID $pid on port $ROS_TCP_PORT" || true
fi
sleep 2

# 等待端口确实释放
for i in 1 2 3 4 5; do
  if ! ss -tlnp 2>/dev/null | grep -q ":$ROS_TCP_PORT "; then
    break
  fi
  echo "[run_tcp_endpoint] Waiting for port $ROS_TCP_PORT..."
  sleep 1
done

source /opt/ros/humble/setup.bash
source "$DRONE_WS/install/setup.bash"
exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$ROS_IP" -p ROS_TCP_PORT:="$ROS_TCP_PORT"
