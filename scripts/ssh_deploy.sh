#!/bin/bash
# ============================================================================
# ssh_deploy.sh — Deploy Project Files to Raspberry Pi via SCP
#
# Copies the src/ and config/ directories to the RPi over SSH/WiFi.
#
# Usage (from your Windows/Mac/Linux laptop):
#   ./scripts/ssh_deploy.sh [PI_IP] [PI_USER]
#
# Defaults:
#   PI_IP   = 192.168.1.100
#   PI_USER = pi
# ============================================================================

PI_IP="${1:-192.168.137.205}"
PI_USER="${2:-ascend}"
REMOTE_DIR="/home/${PI_USER}/ascend"

echo "========================================================"
echo "  ASCEND — Deploying to Raspberry Pi"
echo "  Target: ${PI_USER}@${PI_IP}:${REMOTE_DIR}"
echo "========================================================"

# Create remote directory
echo "[1/3] Creating remote directory..."
ssh "${PI_USER}@${PI_IP}" "mkdir -p ${REMOTE_DIR}/src ${REMOTE_DIR}/config"

# Copy source files
echo "[2/3] Copying source files..."
scp -r src/*.py "${PI_USER}@${PI_IP}:${REMOTE_DIR}/src/"
scp requirements.txt "${PI_USER}@${PI_IP}:${REMOTE_DIR}/"

# Copy config files
echo "[3/3] Copying config files..."
scp -r config/* "${PI_USER}@${PI_IP}:${REMOTE_DIR}/config/" 2>/dev/null || true

echo ""
echo "========================================================"
echo "  Deployment complete!"
echo "  To start mission: ./scripts/ssh_start_mission.sh ${PI_IP} ${PI_USER}"
echo "========================================================"
