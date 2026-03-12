"""Syntax check all Python files and run offline tests."""
import ast
import os
import sys

PROJECT = os.path.dirname(os.path.abspath(__file__))
os.chdir(PROJECT)

# --- Syntax Check ---
print("=" * 60)
print("  SYNTAX CHECK")
print("=" * 60)

files = [
    "src/__init__.py", "src/config.py", "src/mavlink_manager.py",
    "src/realsense_manager.py", "src/visual_odometry.py",
    "src/ekf_fusion.py", "src/flight_controller.py",
    "src/logger.py", "src/main.py",
    "tests/test_config.py", "tests/test_ekf_fusion.py",
    "tests/test_flight_logic.py",
]

all_ok = True
for f in files:
    try:
        with open(f, encoding="utf-8") as fh:
            ast.parse(fh.read())
        print(f"  OK   {f}")
    except SyntaxError as e:
        print(f"  FAIL {f} -> {e}")
        all_ok = False

# --- Run offline tests ---
print()
print("=" * 60)
print("  UNIT TESTS (no hardware needed)")
print("=" * 60)

sys.path.insert(0, PROJECT)

print("\n--- test_config ---")
try:
    from tests.test_config import (
        test_baud_rate, test_hover_duration, test_vision_rate,
        test_camera_fps, test_altitude_range, test_camera_intrinsics,
        test_ekf_noise, test_mode_numbers,
    )
    test_baud_rate()
    test_hover_duration()
    test_vision_rate()
    test_camera_fps()
    test_altitude_range()
    test_camera_intrinsics()
    test_ekf_noise()
    test_mode_numbers()
    print("  All config tests PASSED")
except Exception as e:
    print(f"  FAILED: {e}")
    all_ok = False

print("\n--- test_ekf_fusion ---")
try:
    from tests.test_ekf_fusion import (
        test_initial_state, test_predict_with_zero_accel,
        test_predict_with_acceleration, test_update_corrects_position,
        test_convergence, test_uncertainty_decreases, test_reset,
    )
    test_initial_state()
    test_predict_with_zero_accel()
    test_predict_with_acceleration()
    test_update_corrects_position()
    test_convergence()
    test_uncertainty_decreases()
    test_reset()
    print("  All EKF tests PASSED")
except Exception as e:
    print(f"  FAILED: {e}")
    import traceback; traceback.print_exc()
    all_ok = False

print("\n--- test_flight_logic ---")
try:
    from tests.test_flight_logic import (
        test_initial_state as fl_init, test_init_to_waiting,
        test_waiting_stays_in_stabilize, test_loiter_starts_hovering,
        test_hover_timer, test_land_command, test_landing_to_landed,
        test_full_state_sequence,
    )
    fl_init()
    test_init_to_waiting()
    test_waiting_stays_in_stabilize()
    test_loiter_starts_hovering()
    test_hover_timer()
    test_land_command()
    test_landing_to_landed()
    test_full_state_sequence()
    print("  All flight logic tests PASSED")
except Exception as e:
    print(f"  FAILED: {e}")
    import traceback; traceback.print_exc()
    all_ok = False

print()
print("=" * 60)
if all_ok:
    print("  ALL CHECKS PASSED!")
else:
    print("  SOME CHECKS FAILED - see above")
print("=" * 60)
