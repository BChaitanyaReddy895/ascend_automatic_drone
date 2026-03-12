"""
test_flight_logic.py — Tests for the Flight State Machine.

Tests state transitions without requiring any hardware.
Uses a mock MAVLinkManager.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time
from src.flight_controller import FlightController, FlightState
from src.config import MODE_STABILIZE, MODE_LOITER, MODE_LAND


class MockMAVLinkManager:
    """
    Mock MAVLink manager for testing flight logic without hardware.
    """

    MODE_MAP_REV = {
        MODE_STABILIZE: "STABILIZE",
        MODE_LOITER: "LOITER",
        MODE_LAND: "LAND",
    }

    def __init__(self):
        self._mode = MODE_STABILIZE
        self._armed = True
        self._mode_change_log = []

    def get_flight_mode(self):
        mode_name = self.MODE_MAP_REV.get(self._mode, f"UNKNOWN({self._mode})")
        return self._mode, mode_name

    def set_flight_mode(self, mode_name):
        mode_map = {"STABILIZE": MODE_STABILIZE, "LOITER": MODE_LOITER, "LAND": MODE_LAND}
        self._mode = mode_map.get(mode_name.upper(), self._mode)
        self._mode_change_log.append(mode_name)
        return True

    def is_armed(self):
        return self._armed

    def set_mode(self, mode_num):
        self._mode = mode_num

    def set_armed(self, armed):
        self._armed = armed


def test_initial_state():
    """Flight controller starts in INIT state."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)
    assert fc.get_state() == FlightState.INIT
    print("  PASS: Initial state is INIT")


def test_init_to_waiting():
    """After first tick, should transition from INIT → WAITING_FOR_LOITER."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    state = fc.tick()
    assert state == FlightState.WAITING_FOR_LOITER
    print("  PASS: INIT → WAITING_FOR_LOITER")


def test_waiting_stays_in_stabilize():
    """Should stay in WAITING_FOR_LOITER if mode is STABILIZE."""
    mock_mav = MockMAVLinkManager()
    mock_mav.set_mode(MODE_STABILIZE)
    fc = FlightController(mock_mav)

    fc.tick()  # INIT → WAITING
    state = fc.tick()  # Should stay in WAITING
    assert state == FlightState.WAITING_FOR_LOITER
    print("  PASS: Stays in WAITING while STABILIZE")


def test_loiter_starts_hovering():
    """Switching to LOITER should trigger HOVERING state."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    fc.tick()  # INIT → WAITING

    mock_mav.set_mode(MODE_LOITER)
    state = fc.tick()  # WAITING → HOVERING
    assert state == FlightState.HOVERING
    print("  PASS: LOITER detected → HOVERING")


def test_hover_timer():
    """Hover elapsed time should increase."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    fc.tick()  # INIT → WAITING
    mock_mav.set_mode(MODE_LOITER)
    fc.tick()  # WAITING → HOVERING

    time.sleep(0.2)
    fc.tick()

    assert fc.get_hover_elapsed() > 0.1
    assert fc.get_hover_remaining() > 0
    print(f"  PASS: Hover timer running (elapsed={fc.get_hover_elapsed():.2f}s)")


def test_land_command():
    """After hover timer expires, should command LAND mode."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    # Override hover duration for testing
    import src.flight_controller as fc_module
    original_duration = fc_module.HOVER_DURATION_SEC
    fc_module.HOVER_DURATION_SEC = 0.1  # 100ms for testing

    fc.tick()  # INIT → WAITING
    mock_mav.set_mode(MODE_LOITER)
    fc.tick()  # WAITING → HOVERING

    time.sleep(0.2)  # Wait for timer to expire
    state = fc.tick()  # HOVERING → LANDING

    assert state == FlightState.LANDING
    assert "LAND" in mock_mav._mode_change_log
    print("  PASS: Timer expired → LAND mode commanded")

    # Restore
    fc_module.HOVER_DURATION_SEC = original_duration


def test_landing_to_landed():
    """When drone disarms during LAND, should transition to LANDED."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    import src.flight_controller as fc_module
    original_duration = fc_module.HOVER_DURATION_SEC
    fc_module.HOVER_DURATION_SEC = 0.1

    fc.tick()  # INIT → WAITING
    mock_mav.set_mode(MODE_LOITER)
    fc.tick()  # WAITING → HOVERING

    time.sleep(0.2)
    fc.tick()  # HOVERING → LANDING

    # Simulate drone landing and disarming
    mock_mav.set_armed(False)
    state = fc.tick()

    assert state == FlightState.LANDED
    assert fc.is_mission_complete()
    print("  PASS: Disarmed → LANDED (mission complete)")

    fc_module.HOVER_DURATION_SEC = original_duration


def test_full_state_sequence():
    """Test the complete state sequence: INIT → WAITING → HOVERING → LANDING → LANDED."""
    mock_mav = MockMAVLinkManager()
    fc = FlightController(mock_mav)

    import src.flight_controller as fc_module
    original_duration = fc_module.HOVER_DURATION_SEC
    fc_module.HOVER_DURATION_SEC = 0.1

    states = []

    # INIT → WAITING
    states.append(fc.tick())

    # WAITING (still STABILIZE)
    states.append(fc.tick())

    # Switch to LOITER → HOVERING
    mock_mav.set_mode(MODE_LOITER)
    states.append(fc.tick())

    # Wait for timer
    time.sleep(0.2)

    # HOVERING → LANDING
    states.append(fc.tick())

    # Disarm → LANDED
    mock_mav.set_armed(False)
    states.append(fc.tick())

    expected = [
        FlightState.WAITING_FOR_LOITER,
        FlightState.WAITING_FOR_LOITER,
        FlightState.HOVERING,
        FlightState.LANDING,
        FlightState.LANDED,
    ]

    assert states == expected, f"Expected {expected}, got {states}"
    print("  PASS: Full sequence INIT→WAIT→HOVER→LAND→LANDED verified")

    fc_module.HOVER_DURATION_SEC = original_duration


if __name__ == "__main__":
    print("Flight Logic Tests:")
    test_initial_state()
    test_init_to_waiting()
    test_waiting_stays_in_stabilize()
    test_loiter_starts_hovering()
    test_hover_timer()
    test_land_command()
    test_landing_to_landed()
    test_full_state_sequence()
    print("\nAll flight logic tests passed!")
