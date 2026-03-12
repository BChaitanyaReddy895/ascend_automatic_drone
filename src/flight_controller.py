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

        This should be called at a regular rate (e.g., 10-20 Hz) from
        the main loop. It reads the current Pixhawk mode and transitions
        states accordingly.

        Returns:
            Current FlightState enum value.
        """
        try:
            if self.state == FlightState.INIT:
                self._handle_init()

            elif self.state == FlightState.WAITING_FOR_LOITER:
                self._handle_waiting_for_loiter()

            elif self.state == FlightState.HOVERING:
                self._handle_hovering()

            elif self.state == FlightState.LANDING:
                self._handle_landing()

            elif self.state == FlightState.LANDED:
                pass  # Terminal state

            elif self.state == FlightState.ERROR:
                pass  # Terminal state — needs manual intervention

        except Exception as e:
            self._transition_to(FlightState.ERROR)
            self._error_message = str(e)
            self._log(f"State machine error: {e}", level="ERROR")

        return self.state

    # -------------------------------------------------------------------------
    # State Handlers
    # -------------------------------------------------------------------------

    def _handle_init(self):
        """
        INIT state: Verify MAVLink connection is alive and Pixhawk is responding.
        Transition to WAITING_FOR_LOITER once heartbeat is confirmed.
        """
        mode_num, mode_name = self.mav.get_flight_mode()
        if mode_num is not None:
            self._log(f"Pixhawk connected. Current mode: {mode_name}")
            self._log(f"Waiting for pilot to switch to LOITER mode...")
            self._log(f"(Pilot should take off in STABILIZE to 3-4m, then switch to LOITER)")
            self._transition_to(FlightState.WAITING_FOR_LOITER)
        else:
            self._log("Waiting for Pixhawk heartbeat...", level="DEBUG")

    def _handle_waiting_for_loiter(self):
        """
        WAITING_FOR_LOITER: Continuously poll Pixhawk mode.
        When LOITER is detected, start the 60-second hover timer.
        """
        mode_num, mode_name = self.mav.get_flight_mode()

        if mode_name == "LOITER":
            self._hover_start_time = time.time()
            self._log("=" * 60)
            self._log(f"LOITER MODE DETECTED! Starting {HOVER_DURATION_SEC}s hover timer.")
            self._log("=" * 60)
            self._transition_to(FlightState.HOVERING)
        else:
            # Still waiting — periodically log the current mode
            pass

    def _handle_hovering(self):
        """
        HOVERING: Count down the 60-second hover timer.
        Vision position estimates continue being sent by the vision thread.
        When timer expires, command LAND mode.
        """
        self._hover_elapsed = time.time() - self._hover_start_time
        remaining = HOVER_DURATION_SEC - self._hover_elapsed

        if remaining <= 0:
            # Timer expired — command landing
            self._log("=" * 60)
            self._log(f"HOVER TIMER COMPLETE ({HOVER_DURATION_SEC}s)! Commanding LAND mode.")
            self._log("=" * 60)

            success = self.mav.set_flight_mode("LAND")
            if success:
                self._log("LAND mode command accepted by Pixhawk.")
                self._transition_to(FlightState.LANDING)
            else:
                self._log("LAND mode command FAILED — retrying...", level="WARNING")
                # Will retry on next tick
        else:
            # Log countdown every 10 seconds
            if int(remaining) % 10 == 0 and abs(remaining - int(remaining)) < 0.1:
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
