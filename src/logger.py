"""
logger.py — Mission Logging System

Provides:
  - Timestamped console logging with colored levels
  - CSV telemetry logging (pose, mode, battery, state)
  - Log file rotation
"""

import os
import time
import csv
import datetime
import threading

from src.config import (
    LOG_DIRECTORY,
    LOG_LEVEL,
    LOG_TO_CSV,
    LOG_CSV_RATE_HZ,
)


# Log level priority
LOG_LEVELS = {
    "DEBUG": 0,
    "INFO": 1,
    "WARNING": 2,
    "ERROR": 3,
    "CRITICAL": 4,
}


class MissionLogger:
    """
    Timestamped mission logger with console + CSV output.

    Logs state transitions, pose estimates, flight mode changes,
    battery status, and errors.
    """

    def __init__(self, session_name=None):
        """
        Args:
            session_name: Optional name for this session (used in filename).
                          Defaults to timestamp-based name.
        """
        self._min_level = LOG_LEVELS.get(LOG_LEVEL, 1)
        self._lock = threading.Lock()

        # Create log directory
        os.makedirs(LOG_DIRECTORY, exist_ok=True)

        # Session name for filenames
        if session_name is None:
            session_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_name = session_name

        # Text log file
        self._log_filepath = os.path.join(
            LOG_DIRECTORY, f"mission_{session_name}.log"
        )
        self._log_file = open(self._log_filepath, "a", buffering=1)

        # CSV telemetry file
        self._csv_filepath = None
        self._csv_file = None
        self._csv_writer = None
        self._last_csv_time = 0.0
        self._csv_interval = 1.0 / LOG_CSV_RATE_HZ if LOG_CSV_RATE_HZ > 0 else 1.0

        if LOG_TO_CSV:
            self._csv_filepath = os.path.join(
                LOG_DIRECTORY, f"telemetry_{session_name}.csv"
            )
            self._csv_file = open(self._csv_filepath, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "elapsed_s",
                "state", "flight_mode",
                "x", "y", "z",
                "vx", "vy", "vz",
                "hover_remaining",
                "battery_v", "battery_pct",
                "source", "message",
            ])

        self._start_time = time.time()

        self.log(f"Logger initialized. Log: {self._log_filepath}")
        if self._csv_filepath:
            self.log(f"CSV telemetry: {self._csv_filepath}")

    # -------------------------------------------------------------------------
    # Text Logging
    # -------------------------------------------------------------------------

    def log(self, message, level="INFO", source="System"):
        """
        Log a timestamped message to console and log file.

        Args:
            message: Log message string.
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
            source: Source module name (e.g., "MAVLink", "EKF").
        """
        level_num = LOG_LEVELS.get(level, 1)
        if level_num < self._min_level:
            return

        elapsed = time.time() - self._start_time
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

        line = f"[{timestamp}] [{elapsed:8.2f}s] [{source:12s}] [{level:8s}] {message}"

        with self._lock:
            # Console output
            print(line, flush=True)

            # File output
            try:
                self._log_file.write(line + "\n")
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # CSV Telemetry Logging
    # -------------------------------------------------------------------------

    def log_telemetry(self, state="", flight_mode="",
                      x=0.0, y=0.0, z=0.0,
                      vx=0.0, vy=0.0, vz=0.0,
                      hover_remaining=0.0,
                      battery_v=0.0, battery_pct=0.0,
                      source="", message=""):
        """
        Log a telemetry row to the CSV file.

        Rate-limited to LOG_CSV_RATE_HZ to avoid huge files.

        Args:
            state: Current flight state name.
            flight_mode: Current Pixhawk flight mode.
            x, y, z: Position in meters.
            vx, vy, vz: Velocity in m/s.
            hover_remaining: Seconds remaining in hover.
            battery_v: Battery voltage.
            battery_pct: Battery percentage.
            source: Source module.
            message: Optional note.
        """
        if not self._csv_writer:
            return

        now = time.time()
        if now - self._last_csv_time < self._csv_interval:
            return

        self._last_csv_time = now
        elapsed = now - self._start_time

        with self._lock:
            try:
                self._csv_writer.writerow([
                    datetime.datetime.now().isoformat(),
                    f"{elapsed:.3f}",
                    state, flight_mode,
                    f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                    f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}",
                    f"{hover_remaining:.1f}",
                    f"{battery_v:.2f}", f"{battery_pct:.1f}",
                    source, message,
                ])
                self._csv_file.flush()
            except Exception as e:
                print(f"[Logger] CSV write error: {e}")

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def close(self):
        """Flush and close all log files."""
        self.log("Logger shutting down.")
        with self._lock:
            try:
                self._log_file.flush()
                self._log_file.close()
            except Exception:
                pass
            try:
                if self._csv_file:
                    self._csv_file.flush()
                    self._csv_file.close()
            except Exception:
                pass

    def __del__(self):
        """Destructor — ensure files are closed."""
        try:
            self.close()
        except Exception:
            pass
