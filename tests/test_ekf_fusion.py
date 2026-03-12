"""
test_ekf_fusion.py — Tests for the Extended Kalman Filter.

Tests the EKF with synthetic IMU + visual odometry data
(no hardware required).
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from src.ekf_fusion import EKFFusion


def test_initial_state():
    """EKF should start at origin with zero velocity."""
    ekf = EKFFusion()
    pos = ekf.get_position()
    vel = ekf.get_velocity()

    assert np.allclose(pos, [0, 0, 0]), f"Expected [0,0,0], got {pos}"
    assert np.allclose(vel, [0, 0, 0]), f"Expected [0,0,0], got {vel}"
    print("  PASS: Initial state is zero")


def test_predict_with_zero_accel():
    """With no acceleration (gravity compensated), state shouldn't change much."""
    ekf = EKFFusion()

    # Predict with zero acceleration (after gravity compensation)
    accel = np.array([0.0, 0.0, 9.81])  # only gravity
    ekf.predict(accel, dt=0.05)

    pos = ekf.get_position()
    # Position should remain near zero (gravity is subtracted internally)
    assert np.allclose(pos, [0, 0, 0], atol=0.01), f"Expected near-zero, got {pos}"
    print("  PASS: Zero-accel prediction stays near origin")


def test_predict_with_acceleration():
    """EKF should integrate acceleration into velocity and position."""
    ekf = EKFFusion()

    # Apply 1 m/s² in X direction (plus gravity on Z) for 1 second
    accel = np.array([1.0, 0.0, 9.81])
    dt = 0.05
    steps = 20  # 20 * 0.05 = 1.0 second

    for _ in range(steps):
        ekf.predict(accel, dt)

    vel = ekf.get_velocity()
    pos = ekf.get_position()

    # After 1s at 1 m/s²: v = 1 m/s, x = 0.5 m
    assert vel[0] > 0.5, f"Expected vx > 0.5, got {vel[0]}"
    assert pos[0] > 0.2, f"Expected x > 0.2, got {pos[0]}"
    print(f"  PASS: Acceleration integrates correctly (x={pos[0]:.3f}, vx={vel[0]:.3f})")


def test_update_corrects_position():
    """EKF update should pull state toward measurement."""
    ekf = EKFFusion()

    # Set EKF state to origin
    # Then update with a measurement at (1, 2, 3)
    measurement = np.array([1.0, 2.0, 3.0])
    ekf.update(measurement)

    pos = ekf.get_position()
    # After update, position should move toward measurement
    assert pos[0] > 0.3, f"Expected x > 0.3 after update, got {pos[0]}"
    assert pos[1] > 0.6, f"Expected y > 0.6 after update, got {pos[1]}"
    assert pos[2] > 0.9, f"Expected z > 0.9 after update, got {pos[2]}"
    print(f"  PASS: Update corrects toward measurement (pos={pos})")


def test_convergence():
    """Multiple updates at the same position should converge."""
    ekf = EKFFusion()
    target = np.array([5.0, 3.0, 2.0])

    for _ in range(50):
        ekf.predict(np.array([0.0, 0.0, 9.81]), dt=0.05)
        ekf.update(target)

    pos = ekf.get_position()
    assert np.allclose(pos, target, atol=0.5), f"Expected near {target}, got {pos}"
    print(f"  PASS: Converges to target (pos={pos}, target={target})")


def test_uncertainty_decreases():
    """Uncertainty should decrease with more measurements."""
    ekf = EKFFusion()

    unc_before = ekf.get_uncertainty()

    for _ in range(10):
        ekf.predict(np.array([0.0, 0.0, 9.81]), dt=0.05)
        ekf.update(np.array([1.0, 1.0, 1.0]))

    unc_after = ekf.get_uncertainty()
    assert np.all(unc_after < unc_before), \
        f"Uncertainty should decrease: {unc_before} → {unc_after}"
    print(f"  PASS: Uncertainty decreased ({unc_before} → {unc_after})")


def test_reset():
    """Reset should return to origin."""
    ekf = EKFFusion()
    ekf.update(np.array([10, 20, 30]))
    ekf.reset()

    pos = ekf.get_position()
    assert np.allclose(pos, [0, 0, 0]), f"Expected [0,0,0] after reset, got {pos}"
    print("  PASS: Reset returns to origin")


if __name__ == "__main__":
    print("EKF Fusion Tests:")
    test_initial_state()
    test_predict_with_zero_accel()
    test_predict_with_acceleration()
    test_update_corrects_position()
    test_convergence()
    test_uncertainty_decreases()
    test_reset()
    print("\nAll EKF tests passed!")
