"""
mavlink_manager.py — MAVLink Communication with Pixhawk

Handles:
  - Serial connection to Pixhawk via /dev/serial0 (Telem2)
  - Heartbeat waiting and sending
  - Flight mode monitoring and switching
  - Sending VISION_POSITION_ESTIMATE messages
"""

import time
import threading
from pymavlink import mavutil

from src.config import (
    MAVLINK_SERIAL_PORT,
    MAVLINK_BAUD_RATE,
    COMPANION_SYSID,
    COMPANION_COMPID,
    HEARTBEAT_INTERVAL,
    MODE_STABILIZE,
    MODE_LOITER,
    MODE_LAND,
    MODE_GUIDED,
)


class MAVLinkManager:
    """
    Thread-safe MAVLink manager for Pixhawk communication.
    
    Uses a dedicated background thread for reading all incoming messages
    to avoid serial port conflicts between threads.
    """

    # ArduPilot flight mode name → number mapping
    MODE_MAP = {
        "STABILIZE": MODE_STABILIZE,
        "LOITER": MODE_LOITER,
        "LAND": MODE_LAND,
        "GUIDED": MODE_GUIDED,
    }

    # Reverse mapping: number → name
    MODE_MAP_REV = {v: k for k, v in MODE_MAP.items()}

    def __init__(self, logger=None):
        """
        Args:
            logger: Optional logger instance for status messages.
        """
        self.connection = None
        self.logger = logger
        
        # Threading control
        self._lock = threading.Lock()
        self._running = False
        self._threads = []
        
        # Cached telemetry state
        self._current_mode = None
        self._mode_name = "UNKNOWN"
        self._armed = False
        self._battery_v = 0.0
        self._battery_pct = 0.0
        self._last_heartbeat = 0.0

    # -------------------------------------------------------------------------
    # Connection
    # -------------------------------------------------------------------------

    def connect(self, timeout=30):
        """
        Open serial MAVLink connection and start background threads.
        """
        self._log(f"Connecting to Pixhawk on {MAVLINK_SERIAL_PORT} at {MAVLINK_BAUD_RATE} baud...")

        try:
            self.connection = mavutil.mavlink_connection(
                MAVLINK_SERIAL_PORT,
                baud=MAVLINK_BAUD_RATE,
                source_system=COMPANION_SYSID,
                source_component=COMPANION_COMPID,
            )
        except Exception as e:
            self._log(f"CRITICAL: Failed to open serial port: {e}", level="ERROR")
            return False

        self._log("Waiting for initial Pixhawk heartbeat...")
        heartbeat = self.connection.wait_heartbeat(timeout=timeout)

        if heartbeat is None:
            self._log("ERROR: No heartbeat received — check wiring!", level="ERROR")
            return False

        self._update_heartbeat_data(heartbeat)
        self._log(f"Heartbeat received! Mode: {self._mode_name}, Armed: {self._armed}")

        # Start background threads
        self._running = True
        
        # 1. Listener thread (Reads everything)
        t_listener = threading.Thread(target=self._listener_loop, daemon=True, name="MAVListener")
        t_listener.start()
        self._threads.append(t_listener)
        
        # 2. Heartbeat thread (Sends 1Hz heartbeat)
        t_heartbeat = threading.Thread(target=self._heartbeat_loop, daemon=True, name="MAVHeartbeat")
        t_heartbeat.start()
        self._threads.append(t_heartbeat)

        return True

    def disconnect(self):
        """Cleanly shut down MAVLink connection and threads."""
        self._running = False
        for t in self._threads:
            if t.is_alive():
                t.join(timeout=2)
        
        with self._lock:
            if self.connection:
                self.connection.close()
        self._log("MAVLink connection closed.")

    # -------------------------------------------------------------------------
    # Background Threads
    # -------------------------------------------------------------------------

    def _listener_loop(self):
        """
        Single thread responsible for ALL reading from the serial port.
        Updates internal state caches for other threads to read.
        """
        while self._running:
            try:
                # Blocking read for any message
                msg = self.connection.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    continue

                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    self._update_heartbeat_data(msg)
                elif msg_type == "SYS_STATUS":
                    self._battery_v = msg.voltage_battery / 1000.0
                    self._battery_pct = msg.battery_remaining
                elif msg_type == "STATUSTEXT":
                    self._log(f"Pixhawk: {msg.text}", level="INFO")

            except Exception as e:
                # Don't log read errors too often to avoid spam
                pass

    def _heartbeat_loop(self):
        """Continuously send heartbeat to Pixhawk at 1 Hz (Thread-safe)."""
        while self._running:
            try:
                with self._lock:
                    self.connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
            except Exception as e:
                self._log(f"Heartbeat send error: {e}", level="WARNING")
            time.sleep(HEARTBEAT_INTERVAL)

    def _update_heartbeat_data(self, msg):
        """Helper to parse heartbeat and update local state."""
        self._current_mode = msg.custom_mode
        self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self._mode_name = self.MODE_MAP_REV.get(self._current_mode, f"UNKNOWN({self._current_mode})")
        self._last_heartbeat = time.time()

    # -------------------------------------------------------------------------
    # State Getters (Non-blocking, uses Cache)
    # -------------------------------------------------------------------------

    def get_flight_mode(self):
        """Return the latest cached flight mode."""
        return self._current_mode, self._mode_name

    def is_mode(self, mode_name):
        """Check if cached mode matches name."""
        return self._mode_name == mode_name.upper()

    def is_armed(self):
        """Return whether the drone is currently armed (cached)."""
        return self._armed

    def get_battery_status(self):
        """Return the latest cached battery status."""
        return self._battery_v, self._battery_pct

    # -------------------------------------------------------------------------
    # Commands (Thread-safe Writes)
    # -------------------------------------------------------------------------

    def set_flight_mode(self, mode_name):
        """Command Pixhawk to switch mode (Thread-safe)."""
        mode_num = self.MODE_MAP.get(mode_name.upper())
        if mode_num is None:
            return False

        self._log(f"Switching flight mode to {mode_name}...")
        try:
            with self._lock:
                self.connection.mav.set_mode_send(
                    self.connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_num,
                )
            return True
        except Exception as e:
            self._log(f"Mode change send error: {e}", level="ERROR")
            return False

    def send_vision_position_estimate(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Send VISION_POSITION_ESTIMATE (Thread-safe)."""
        timestamp_us = int(time.time() * 1e6)
        try:
            with self._lock:
                self.connection.mav.vision_position_estimate_send(
                    timestamp_us, x, y, z, roll, pitch, yaw
                )
        except Exception:
            pass

    def request_data_stream(self, stream_id=mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz=10):
        """Request streams (Thread-safe)."""
        try:
            with self._lock:
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    stream_id, rate_hz, 1
                )
        except Exception:
            pass

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="MAVLink")
        else:
            print(f"[MAVLink] [{level}] {message}")
