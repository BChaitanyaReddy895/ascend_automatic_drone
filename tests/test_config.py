"""
test_config.py — Tests for configuration constants.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.config import (
    MAVLINK_BAUD_RATE,
    HOVER_DURATION_SEC,
    VISION_SEND_RATE_HZ,
    RS_DEPTH_FPS,
    RS_IR_FPS,
    TARGET_ALTITUDE_MIN,
    TARGET_ALTITUDE_MAX,
    CAMERA_FX,
    CAMERA_FY,
    EKF_PROCESS_NOISE_POS,
    EKF_MEASUREMENT_NOISE_VO,
    MODE_STABILIZE,
    MODE_LOITER,
    MODE_LAND,
)


def test_baud_rate():
    """Baud rate must be 921600 to match Pixhawk Telem2 config."""
    assert MAVLINK_BAUD_RATE == 921600


def test_hover_duration():
    """Hover must be positive and reasonable."""
    assert 10 <= HOVER_DURATION_SEC <= 600


def test_vision_rate():
    """Vision position rate must be >= 20 Hz as required."""
    assert VISION_SEND_RATE_HZ >= 20


def test_camera_fps():
    """Camera streams must be positive."""
    assert RS_DEPTH_FPS > 0
    assert RS_IR_FPS > 0


def test_altitude_range():
    """Target altitude must be valid."""
    assert TARGET_ALTITUDE_MIN > 0
    assert TARGET_ALTITUDE_MAX > TARGET_ALTITUDE_MIN
    assert TARGET_ALTITUDE_MIN >= 3.0
    assert TARGET_ALTITUDE_MAX <= 5.0


def test_camera_intrinsics():
    """Focal lengths must be positive."""
    assert CAMERA_FX > 0
    assert CAMERA_FY > 0


def test_ekf_noise():
    """Noise values must be positive."""
    assert EKF_PROCESS_NOISE_POS > 0
    assert EKF_MEASUREMENT_NOISE_VO > 0


def test_mode_numbers():
    """ArduPilot mode numbers must be correct."""
    assert MODE_STABILIZE == 0
    assert MODE_LOITER == 5
    assert MODE_LAND == 9


if __name__ == "__main__":
    test_baud_rate()
    test_hover_duration()
    test_vision_rate()
    test_camera_fps()
    test_altitude_range()
    test_camera_intrinsics()
    test_ekf_noise()
    test_mode_numbers()
    print("All config tests passed!")
