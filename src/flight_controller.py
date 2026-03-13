"""
flight_controller.py — Flight Logic State Machine

Manages the autonomous hover and landing sequence:
  1. WAITING_FOR_LOITER: Monitors Pixhawk mode, waits for pilot to switch to LOITER
  2. HOVERING: Runs 60-second timer while ensuring VISION_POSITION_ESTIMATE continues
  3. LANDING: Commands LAND mode, continues sending vision data during descent
  4. LANDED: Mission complete, drone is on the ground
"""

import time
import enum

from src.config import (
    HOVER_DURATION_SEC,
    LANDING_ALTITUDE_THRESHOLD,
    LANDING_VELOCITY_THRESHOLD,
)


class FlightState(enum.Enum):
    """Flight state machine states."""
    INIT = "INIT"
    WAITING_FOR_LOITER = "WAITING_FOR_LOITER"
    HOVERING = "HOVERING"
    LANDING = "LANDING"
    LANDED = "LANDED"
    ERROR = "ERROR"


class FlightController:
    """
    Autonomous flight logic controller.

    Monitors the Pixhawk flight mode via MAVLinkManager and controls
    the hover-and-land sequence.
    """

    def __init__(self, mavlink_manager, logger=None):
        """
        Args:
            mavlink_manager: MAVLinkManager instance for mode monitoring/switching.
            logger: Optional logger instance.
        """
        self.mav = mavlink_manager
        self.logger = logger

        self.state = FlightState.INIT
        self._hover_start_time = None
        self._hover_elapsed = 0.0
        self._mission_complete = False
        self._error_message = ""

    # -------------------------------------------------------------------------
    # Main Tick (called every loop iteration)
    # -------------------------------------------------------------------------

    def tick(self):
        """
        Execute one iteration of the flight state machine.
        (Non-blocking, called periodically)
        """
        try:
            # Always poll the current mode first
            _, mode_name = self.mav.get_flight_mode()

            if self.state == FlightState.INIT:
                self._handle_init(mode_name)

            elif self.state == FlightState.WAITING_FOR_LOITER:
                if mode_name == "LOITER":
                    # Transition to HOVERING (Timer starts fresh)
                    self._hover_start_time = time.time()
                    self._log("=" * 60)
                    self._log(f"LOITER MODE DETECTED! Starting {HOVER_DURATION_SEC}s hover timer.")
                    self._log("=" * 60)
                    self._transition_to(FlightState.HOVERING)
                
            elif self.state == FlightState.HOVERING:
                if mode_name != "LOITER":
                    # Pilot aborted or switched out of Loiter
                    self._log(f"Pilot switched out of LOITER (Current: {mode_name}). Resetting mission.")
                    self._transition_to(FlightState.WAITING_FOR_LOITER)
                    self._hover_start_time = None
                else:
                    self._handle_hovering()

            elif self.state == FlightState.LANDING:
                # ArduPilot auto-disarms on land
                if not self.mav.is_armed():
                    self._log("Mission Successful! Drone disarmed.")
                    self._transition_to(FlightState.LANDED)

        except Exception as e:
            self._log(f"State machine error: {e}", level="ERROR")
            self._transition_to(FlightState.ERROR)

        return self.state

    def _handle_init(self, mode_name):
        if mode_name:
            self._log(f"Init Complete. Mode: {mode_name}. Ready for LOITER.")
            self._transition_to(FlightState.WAITING_FOR_LOITER)

    def _handle_hovering(self):
        """Count down the hover timer (Non-blocking)."""
        elapsed = time.time() - self._hover_start_time
        remaining = HOVER_DURATION_SEC - elapsed

        if remaining <= 0:
            self._log("Hover complete. Commanding LAND...")
            if self.mav.set_flight_mode("LAND"):
                self._transition_to(FlightState.LANDING)
        else:
            # Periodic logging
            if int(elapsed) > self._hover_elapsed:
                self._hover_elapsed = int(elapsed)
                if self._hover_elapsed % 10 == 0:
                    self._log(f"Hovering... {int(remaining)}s remaining")

    def _handle_landing(self):
        """
        LANDING: Drone is descending in LAND mode.
        Continue sending vision data to maintain horizontal stability.
        Detect when the drone has touched down.
        """
        mode_num, mode_name = self.mav.get_flight_mode()

        # Check if drone is still in LAND mode
        if mode_name != "LAND":
            self._log(f"WARNING: Mode changed from LAND to {mode_name}!", level="WARNING")
            # If pilot overrides, respect it
            if mode_name == "STABILIZE":
                self._log("Pilot took manual control. Mission aborted.")
                self._transition_to(FlightState.LANDED)
                return

        # Check if armed — when drone lands, ArduPilot auto-disarms
        if not self.mav.is_armed():
            self._log("=" * 60)
            self._log("DRONE DISARMED — Landing complete! Mission successful!")
            self._log("=" * 60)
            self._mission_complete = True
            self._transition_to(FlightState.LANDED)

    # -------------------------------------------------------------------------
    # State Transitions
    # -------------------------------------------------------------------------

    def _transition_to(self, new_state):
        """Log and execute a state transition."""
        old_state = self.state
        self.state = new_state
        self._log(f"State transition: {old_state.value} -> {new_state.value}")

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def is_mission_complete(self):
        """Return True if the mission has finished (landed successfully)."""
        return self._mission_complete

    def get_state(self):
        """Return the current FlightState."""
        return self.state

    def get_hover_elapsed(self):
        """Return seconds elapsed in hover, or 0 if not hovering."""
        return self._hover_elapsed

    def get_hover_remaining(self):
        """Return seconds remaining in hover, or 0 if not hovering."""
        if self.state == FlightState.HOVERING and self._hover_start_time:
            return max(0, HOVER_DURATION_SEC - (time.time() - self._hover_start_time))
        return 0.0

    def get_error(self):
        """Return error message if in ERROR state."""
        return self._error_message

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="FlightCtrl")
        else:
            print(f"[FlightCtrl] [{level}] {message}")
