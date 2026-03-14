"""
config.py — Central Configuration for the ASCEND Drone System

All tunable parameters for MAVLink, RealSense, EKF, flight logic,
and SSH/WiFi settings are defined here.
"""

import os

# =============================================================================
# MAVLink / Pixhawk Connection
# =============================================================================

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
# Intel RealSense D455 Configuration
# =============================================================================

# Resolution & Framerate (Native D455 480p profile)
RS_DEPTH_WIDTH = 640
RS_DEPTH_HEIGHT = 480    # 480p is a native, stable profile for the D455
RS_DEPTH_FPS = 30

RS_IR_WIDTH = 640
RS_IR_HEIGHT = 480       # 480p is a native, stable profile for the D455
RS_IR_FPS = 30

# IMU streams (Maximized for higher FPS stability)
RS_ACCEL_FPS = 250       # Hardware native: 63, 250
RS_GYRO_FPS = 400        # Hardware native: 200, 400

# Image Processing
VO_USE_CLAHE = True      # Enhance contrast for plain tiles
VO_CLAHE_CLIP = 3.0      # Contrast limit
VO_CLAHE_GRID = (8, 8)   # Grid size for adaptive equalization

# =============================================================================
# Visual Odometry (Optical Flow)
# =============================================================================

# Lucas-Kanade optical flow parameters
LK_WIN_SIZE = (25, 25)
LK_MAX_LEVEL = 4
LK_CRITERIA = {
    "type": "EPS_COUNT",
    "max_iter": 30,
    "epsilon": 0.01,
}

# Feature detection (Shi-Tomasi corners)
FEATURE_MAX_CORNERS = 500      # High count for better median stability
FEATURE_QUALITY_LEVEL = 0.0003  # VERY sensitive — track even micro-texture
FEATURE_MIN_DISTANCE = 4       # Pack features closer if needed
FEATURE_BLOCK_SIZE = 7

# Corrected Feature Params for tracker
FEATURE_PARAMS = {
    "maxCorners": FEATURE_MAX_CORNERS,
    "qualityLevel": FEATURE_QUALITY_LEVEL,
    "minDistance": FEATURE_MIN_DISTANCE,
    "blockSize": FEATURE_BLOCK_SIZE,
}

# Visual Odometry Calibration
VO_XY_SCALE = 1.0              # Increase this if drone under-corrects
VO_Z_SCALE = 0.7               # Corrected for 3m vs 14m reporting
VO_MIN_ALTITUDE = 0.4          # Meters
VO_MAX_ALTITUDE = 6.0          # Meters (Limit to avoid ceiling/reflection noise)

# Orientation Calibration (Fixes Drift Directions)
VO_INVERT_X = False            # Set True if Forward/Backward is flipped
VO_INVERT_Y = False            # Set True if Left/Right is flipped
VO_SWAP_XY = False             # Set True if X/Y axes are swapped

# Minimum number of tracked features before re-detection
MIN_FEATURES_THRESHOLD = 25
MIN_GOOD_FLOW_FRACTION = 0.35
MAX_FLOW_MAG_PIX = 40.0
MAX_DZ_M = 0.8    # Re-detect sooner to maintain density

# Camera intrinsics (Native 640x480 center)
CAMERA_FX = 387.5
CAMERA_FY = 387.5
CAMERA_CX = 320.7
CAMERA_CY = 240.0

# =============================================================================
# EKF Sensor Fusion
# =============================================================================

# Process noise covariance
EKF_PROCESS_NOISE_POS = 0.015
EKF_PROCESS_NOISE_VEL = 3.0        # Increased for much faster reactiveness
                                   # (Prevents "late" feeling in telemetry)

# Measurement noise covariance
EKF_MEASUREMENT_NOISE_VO = 0.03    # Trust vision more for real-time tracking
EKF_MEASUREMENT_NOISE_IMU = 0.02

# Initial state uncertainty
EKF_INITIAL_COVARIANCE = 1.0

# =============================================================================
# Flight Logic (ArduPilot Friendly)
# =============================================================================

# Mission Parameters
HOVER_DURATION_SEC = 60.0
VISION_SEND_RATE_HZ = 20           # Continuous data stream rate

# Target altitude range (meters) — pilot takes off manually to this
TARGET_ALTITUDE_MIN = 3.0
TARGET_ALTITUDE_MAX = 4.0

# Mode Definitions
MODE_STABILIZE = 0
MODE_LOITER = 5
MODE_LAND = 9
MODE_GUIDED = 4

# Landing Thresholds
LANDING_ALTITUDE_THRESHOLD = 0.15
LANDING_VELOCITY_THRESHOLD = 0.05

# =============================================================================
# Logging & Network
# =============================================================================

LOG_DIRECTORY = os.path.expanduser("~/ascend_logs")
LOG_LEVEL = "INFO"
LOG_TO_CSV = True
LOG_CSV_RATE_HZ = 10

RPI_USERNAME = "ascend"
RPI_IP = "192.168.137.205"
RPI_PROJECT_DIR = os.path.expanduser("~/ascend")
