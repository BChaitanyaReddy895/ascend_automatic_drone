#!/bin/bash
# ============================================================================
# setup_rpi.sh — First-Time Raspberry Pi 4B Setup
#
# Run this ONCE on the RPi to install all dependencies, configure WiFi,
# enable SSH, and set up the serial UART for Pixhawk communication.
#
# Usage (via SSH):
#   chmod +x setup_rpi.sh
#   ./setup_rpi.sh
# ============================================================================

set -e

echo "========================================================"
echo "  ASCEND — Raspberry Pi 4B Setup Script"
echo "========================================================"

# ---- System Update ----
echo "[1/7] Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

# ---- Enable SSH (should already be enabled if you're running this via SSH) ----
echo "[2/7] Ensuring SSH is enabled..."
sudo systemctl enable ssh
sudo systemctl start ssh

# ---- Install Python & Dependencies ----
echo "[3/7] Installing Python3 and pip..."
sudo apt-get install -y python3 python3-pip python3-venv python3-dev

echo "[3/7] Installing system libraries for OpenCV..."
sudo apt-get install -y \
    libopencv-dev \
    libhdf5-dev \
    libharfbuzz-dev \
    liblapack-dev \
    gfortran \
    tmux

echo "[3/7] Installing Python packages..."
pip3 install --upgrade pip --break-system-packages
pip3 install --break-system-packages \
    pymavlink \
    opencv-python-headless \
    numpy \
    MAVProxy

# ---- Configure Serial UART ----
echo "[4/7] Configuring UART for Pixhawk communication..."

# Disable serial console (frees /dev/serial0 for MAVLink)
sudo sed -i 's/console=serial0,115200 //g' /boot/cmdline.txt 2>/dev/null || true
sudo sed -i 's/console=serial0,115200//g' /boot/firmware/cmdline.txt 2>/dev/null || true

# Enable UART in config.txt
CONFIG_FILE="/boot/config.txt"
if [ -f "/boot/firmware/config.txt" ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
fi

if ! grep -q "enable_uart=1" "$CONFIG_FILE"; then
    echo "" | sudo tee -a "$CONFIG_FILE" > /dev/null
    echo "# Enable UART for Pixhawk MAVLink" | sudo tee -a "$CONFIG_FILE" > /dev/null
    echo "enable_uart=1" | sudo tee -a "$CONFIG_FILE" > /dev/null
    echo "dtoverlay=disable-bt" | sudo tee -a "$CONFIG_FILE" > /dev/null
fi

# Disable serial-getty service
sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# ---- Create Project Directory ----
echo "[5/7] Creating project directory..."
mkdir -p /home/$USER/ascend
mkdir -p /home/$USER/ascend_logs

# ---- WiFi Configuration ----
echo "[6/7] WiFi configuration..."
echo "  Current WiFi status:"
iwconfig wlan0 2>/dev/null | grep -E "ESSID|Signal" || echo "  No WiFi detected"
echo ""
echo "  To connect to a WiFi network, edit:"
echo "    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf"
echo ""
echo "  Or use raspi-config:"
echo "    sudo raspi-config → System Options → Wireless LAN"
echo ""
echo "  To create a WiFi hotspot (RPi as access point):"
echo "    sudo nmcli dev wifi hotspot ifname wlan0 ssid ASCEND_DRONE password ascend2024"

# ---- Verify Installation ----
echo "[7/7] Verifying installation..."
echo "  Python version: $(python3 --version)"
echo "  pymavlink: $(pip3 show pymavlink 2>/dev/null | grep Version || echo 'NOT FOUND')"
echo "  pyrealsense2: $(pip3 show pyrealsense2 2>/dev/null | grep Version || echo 'NOT FOUND')"
echo "  OpenCV: $(python3 -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo 'NOT FOUND')"
echo "  numpy: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo 'NOT FOUND')"
echo "  Serial port: $(ls -la /dev/serial0 2>/dev/null || echo '/dev/serial0 NOT FOUND')"

echo ""
echo "========================================================"
echo "  Setup complete! REBOOT required for UART changes."
echo "  Run: sudo reboot"
echo "========================================================"
