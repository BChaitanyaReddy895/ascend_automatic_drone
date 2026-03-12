"""
config.py — Central Configuration for the ASCEND Drone System

All tunable parameters for MAVLink, RealSense, EKF, flight logic,
and SSH/WiFi settings are defined here.
"""

# =============================================================================
# MAVLink / Pixhawk Connection
# =============================================================================

import os

# Serial port on Raspberry Pi connected to Pixhawk Telem2
MAVLINK_SERIAL_PORT = "/dev/serial0"
MAVLINK_BAUD_RATE = 921600

# MAVLink system IDs
COMPANION_SYSID = 1       # Companion computer system ID
COMPANION_COMPID = 197     # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
PIXHAWK_SYSID = 1         # Pixhawk system ID (default)
PIXHAWK_COMPID = 1         # Pixhawk component ID (default)

# Heartbeat interval (seconds)
HEARTBEAT_INTERVAL = 1.0

# =============================================================================
# MAVProxy UDP Bridge (for WiFi ground station forwarding)
# =============================================================================

# When MAVProxy runs on the RPi, it bridges serial → UDP
# Your laptop can connect a GCS to this UDP endpoint over WiFi
MAVPROXY_UDP_OUT = "udpout:0.0.0.0:14550"
MAVPROXY_UDP_IN = "udpin:0.0.0.0:14551"

# =============================================================================
# Intel RealSense D455 Configuration
# =============================================================================

# Depth stream
RS_DEPTH_WIDTH = 640
RS_DEPTH_HEIGHT = 360
RS_DEPTH_FPS = 15

# Infrared stream (used for optical flow feature tracking)
RS_IR_WIDTH = 640
RS_IR_HEIGHT = 360
RS_IR_FPS = 15

# IMU streams (accelerometer + gyroscope)
RS_ACCEL_FPS = 63        # Hardware native: 63, 250
RS_GYRO_FPS = 200        # Hardware native: 200, 400

# =============================================================================
# Visual Odometry (Optical Flow)
# =============================================================================

# Lucas-Kanade optical flow parameters
LK_WIN_SIZE = (21, 21)
LK_MAX_LEVEL = 3
LK_CRITERIA = {
    "type": "EPS_COUNT",   # cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT
    "max_iter": 30,
    "epsilon": 0.01,
}

# Feature detection (Shi-Tomasi corners)
FEATURE_MAX_CORNERS = 200
FEATURE_QUALITY_LEVEL = 0.05   # Higher quality for better tracking on tiles
FEATURE_MIN_DISTANCE = 10
FEATURE_BLOCK_SIZE = 7

# Minimum number of tracked features before re-detection
MIN_FEATURES_THRESHOLD = 50

# Camera intrinsics (D455 defaults — update after calibration)
# focal length in pixels, principal point in pixels
CAMERA_FX = 382.0
CAMERA_FY = 382.0
CAMERA_CX = 320.0
CAMERA_CY = 240.0

# =============================================================================
# EKF Sensor Fusion
# =============================================================================

# Process noise covariance (tune based on flight tests)
EKF_PROCESS_NOISE_POS = 0.01       # Position uncertainty growth per step
EKF_PROCESS_NOISE_VEL = 0.5        # Increased for faster convergence without IMU

# Measurement noise covariance
EKF_MEASUREMENT_NOISE_VO = 0.05    # Visual odometry measurement noise
EKF_MEASUREMENT_NOISE_IMU = 0.02   # IMU measurement noise

# Initial state uncertainty
EKF_INITIAL_COVARIANCE = 1.0

# =============================================================================
# Flight Logic
# =============================================================================

# Hover duration in seconds after LOITER mode is detected
HOVER_DURATION_SEC = 60.0

# Target altitude range (meters) — pilot takes off manually to this
TARGET_ALTITUDE_MIN = 3.0
TARGET_ALTITUDE_MAX = 4.0

# Vision position estimate send rate (Hz)
VISION_SEND_RATE_HZ = 20

# Landing detection thresholds
LANDING_ALTITUDE_THRESHOLD = 0.15    # meters — consider landed below this
LANDING_VELOCITY_THRESHOLD = 0.05    # m/s — consider stationary below this

# Flight modes (ArduPilot mode numbers)
MODE_STABILIZE = 0
MODE_LOITER = 5
MODE_LAND = 9
MODE_GUIDED = 4

# =============================================================================
# Logging
# =============================================================================

LOG_DIRECTORY = os.path.expanduser("~/ascend_logs")
LOG_LEVEL = "INFO"           # DEBUG, INFO, WARNING, ERROR
LOG_TO_CSV = True            # Also write telemetry to CSV
LOG_CSV_RATE_HZ = 10         # CSV logging rate

# =============================================================================
# SSH / WiFi / Network
# =============================================================================

# Default RPi network settings (used by deployment scripts)
RPI_USERNAME = "ascend"
RPI_HOSTNAME = "ascend-pi.local"    # Updated to match setup
RPI_IP = "192.168.137.205"          # Updated to match current session
RPI_PROJECT_DIR = os.path.expanduser("~/ascend")

# WiFi hotspot settings (if RPi creates its own network)
WIFI_HOTSPOT_SSID = "ASCEND_DRONE"
WIFI_HOTSPOT_PASSWORD = "ascend2024"
