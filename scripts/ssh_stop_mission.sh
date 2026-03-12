#!/bin/bash
# ============================================================================
# ssh_stop_mission.sh — Emergency Stop the ASCEND Mission via SSH
#
# SSH into RPi and:
#   1. Send SIGINT to main.py (graceful shutdown → will attempt LAND mode)
#   2. Optionally force-kill if graceful shutdown fails
#
# Usage:
#   ./scripts/ssh_stop_mission.sh [PI_IP] [PI_USER]
#   ./scripts/ssh_stop_mission.sh 192.168.1.100 pi --force
# ============================================================================

PI_IP="${1:-192.168.1.100}"
PI_USER="${2:-pi}"
FORCE="${3:-}"
SESSION_NAME="ascend_mission"

echo "========================================================"
echo "  ASCEND — Stopping Mission on RPi"
echo "  Target: ${PI_USER}@${PI_IP}"
echo "========================================================"

if [ "$FORCE" == "--force" ]; then
    echo "[FORCE] Killing all Python processes..."
    ssh "${PI_USER}@${PI_IP}" "pkill -9 -f 'python3 -m src.main' || true"
    ssh "${PI_USER}@${PI_IP}" "tmux kill-session -t ${SESSION_NAME} 2>/dev/null || true"
    ssh "${PI_USER}@${PI_IP}" "tmux kill-session -t mavproxy 2>/dev/null || true"
    echo "  Force kill complete."
else
    echo "[1/2] Sending graceful shutdown (SIGINT)..."
    ssh "${PI_USER}@${PI_IP}" "tmux send-keys -t ${SESSION_NAME} C-c"

    echo "  Waiting 10 seconds for graceful shutdown..."
    sleep 10

    echo "[2/2] Checking if process is still running..."
    RUNNING=$(ssh "${PI_USER}@${PI_IP}" "pgrep -f 'python3 -m src.main' || echo ''")

    if [ -n "$RUNNING" ]; then
        echo "  Process still running. Force killing..."
        ssh "${PI_USER}@${PI_IP}" "pkill -9 -f 'python3 -m src.main' || true"
    fi

    ssh "${PI_USER}@${PI_IP}" "tmux kill-session -t ${SESSION_NAME} 2>/dev/null || true"
    echo "  Shutdown complete."
fi

echo ""
echo "========================================================"
echo "  Mission stopped."
echo "  Logs available at: /home/${PI_USER}/ascend_logs/"
echo "========================================================"
