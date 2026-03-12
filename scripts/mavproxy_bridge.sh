#!/bin/bash
# ============================================================================
# mavproxy_bridge.sh — Start MAVProxy on RPi for WiFi Ground Station Access
#
# Bridges the serial UART connection to Pixhawk → UDP output so your
# laptop can connect Mission Planner / QGroundControl over WiFi.
#
# Usage (run on RPi):
#   ./scripts/mavproxy_bridge.sh [GCS_IP]
#
# Then on your laptop, connect GCS to:
#   UDP 14550
# ============================================================================

GCS_IP="${1:-0.0.0.0}"

echo "========================================================"
echo "  MAVProxy Bridge — Serial to UDP"
echo "  Serial: /dev/serial0 @ 921600 baud"
echo "  UDP Out: ${GCS_IP}:14550"
echo "========================================================"

mavproxy.py \
    --master=/dev/serial0 \
    --baudrate=921600 \
    --out=udp:${GCS_IP}:14550 \
    --source-system=254 \
    --source-component=0 \
    --daemon
