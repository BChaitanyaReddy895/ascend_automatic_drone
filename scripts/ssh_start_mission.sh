#!/bin/bash
# ============================================================================
# ssh_start_mission.sh — Start the ASCEND Mission Remotely via SSH
#
# SSH into the RPi and launch main.py inside a tmux session so it
# keeps running even if your SSH connection drops.
#
# Usage:
#   ./scripts/ssh_start_mission.sh [PI_IP] [PI_USER]
# ============================================================================

PI_IP="${1:-192.168.137.205}"
PI_USER="${2:-ascend}"
REMOTE_DIR="/home/${PI_USER}/ascend"
SESSION_NAME="ascend_mission"

echo "========================================================"
echo "  ASCEND — Starting Mission on RPi"
echo "  Target: ${PI_USER}@${PI_IP}"
echo "========================================================"

# Kill any existing mission session
echo "[1/3] Cleaning up old sessions..."
ssh "${PI_USER}@${PI_IP}" "tmux kill-session -t ${SESSION_NAME} 2>/dev/null || true"

# Optionally start MAVProxy bridge for ground station access
echo "[2/3] Starting MAVProxy bridge (optional GCS forwarding)..."
ssh "${PI_USER}@${PI_IP}" "tmux new-session -d -s mavproxy \
    'mavproxy.py --master=/dev/serial0 --baudrate=921600 \
     --out=udp:0.0.0.0:14550 \
     --daemon 2>/dev/null || echo MAVProxy not needed in direct mode'"

# Start the mission in a tmux session
echo "[3/3] Starting mission..."
ssh "${PI_USER}@${PI_IP}" "tmux new-session -d -s ${SESSION_NAME} \
    'cd ${REMOTE_DIR} && python3 -m src.main 2>&1 | tee /home/${PI_USER}/ascend_logs/latest_run.log'"

echo ""
echo "========================================================"
echo "  Mission started in tmux session '${SESSION_NAME}'"
echo ""
echo "  To monitor live output:"
echo "    ssh ${PI_USER}@${PI_IP} 'tmux attach -t ${SESSION_NAME}'"
echo ""
echo "  To view latest logs:"
echo "    ssh ${PI_USER}@${PI_IP} 'tail -f /home/${PI_USER}/ascend_logs/latest_run.log'"
echo ""
echo "  To stop mission:"
echo "    ./scripts/ssh_stop_mission.sh ${PI_IP} ${PI_USER}"
echo "========================================================"
