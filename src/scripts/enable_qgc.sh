#!/usr/bin/env bash
# Send MAVLink commands to a running PX4 instance (tmux session 'drone', window 'px4')
# so QGroundControl (QGC) on another machine can connect.
# Usage: ./enable_qgc.sh [QGC_IP]
# Prereq: PX4 is running and you see the `pxh>` prompt.

QGC_IP="${1:-10.211.55.2}"
SESSION="${SESSION:-drone}"
WINDOW="${WINDOW:-px4}"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Error: tmux session '$SESSION' not found. Start PX4 first (e.g. ./start_drone.sh), wait for pxh>, then run this script."
  exit 1
fi

echo "Sending mavlink commands to PX4 (session=$SESSION, window=$WINDOW) for QGC at $QGC_IP ..."
tmux send-keys -t "$SESSION:$WINDOW" "mavlink stop-all" Enter
sleep 2
tmux send-keys -t "$SESSION:$WINDOW" "mavlink start -x -u 14550 -r 40000 -t $QGC_IP" Enter
echo "Done. In QGC add UDP link, port 14550."
