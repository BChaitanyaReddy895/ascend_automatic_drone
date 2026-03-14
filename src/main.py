"""
main.py — ASCEND Mission Entry Point

Orchestrates the entire autonomous hover and auto-land mission:
  Thread 1: Vision Pipeline  → RealSense → Optical Flow → EKF → Fused pose
  Thread 2: MAVLink TX        → Sends VISION_POSITION_ESTIMATE at 20 Hz
  Thread 3: Flight Controller → State machine (wait → hover → land)

Usage (run on the Raspberry Pi via SSH):
    python -m src.main

Or deploy and run via:
    ./scripts/ssh_start_mission.sh
"""

import sys
import time
import signal
import threading
import numpy as np

from src.config import (
    VISION_SEND_RATE_HZ,
    HOVER_DURATION_SEC,
)
from src.logger import MissionLogger
from src.mavlink_manager import MAVLinkManager
from src.realsense_manager import RealSenseManager
from src.visual_odometry import VisualOdometry
from src.ekf_fusion import EKFFusion
from src.flight_controller import FlightController, FlightState


# =============================================================================
# Global state
# =============================================================================
shutdown_event = threading.Event()


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n[Main] SIGINT received — initiating graceful shutdown...")
    shutdown_event.set()


# =============================================================================
# Thread 1: Vision Pipeline (RealSense → VO → EKF)
# =============================================================================

def vision_thread(realsense, vo, ekf, logger):
    """
    Vision processing loop running at camera frame rate (~30 Hz).

    Pipeline:
      1. Get IR frame + depth frame from RealSense
      2. Compute optical flow displacement (dx, dy, dz)
      3. Get IMU data from D455
      4. EKF predict (IMU) + update (VO)
      5. Store fused position for MAVLink thread to read
    """
    logger.log("Vision thread started.", source="Vision")

    prev_time = time.time()
    frame_count = 0

    while not shutdown_event.is_set():
        try:
            # 1. Get synchronized camera and IMU data
            ir_frame, depth_frame, timestamp = realsense.get_sensor_data(timeout_ms=1000)
            if ir_frame is None or depth_frame is None:
                continue

            # 2. Compute visual odometry
            dx, dy, dz = vo.update(ir_frame, depth_frame)

            if abs(dx) > 1.2 or abs(dy) > 1.2 or abs(dz) > 0.8:
                logger.log("Large VO step - possible outlier - zeroing", source="Vision", level="WARNING")
                dx = dy = dz = 0.0

            # 3. Get IMU data (now updated internally by get_sensor_data)
            accel, gyro, imu_ts = realsense.get_imu_data()

            # 4. EKF predict + update
            now = time.time()
            dt = now - prev_time
            prev_time = now

            if dt > 0 and dt < 0.5:
                ekf.predict(accel, dt)

            vo_pos = vo.get_position()
            ekf.update(vo_pos)

            # Performance logging
            frame_count += 1
            if frame_count % 100 == 0:
                pos = ekf.get_position()
                vel = ekf.get_velocity()
                unc = ekf.get_uncertainty()
                logger.log(
                    f"Frame {frame_count}: "
                    f"pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                    f"vel=({vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}) "
                    f"unc=({unc[0]:.3f}, {unc[1]:.3f}, {unc[2]:.3f})",
                    source="Vision",
                )

        except Exception as e:
            logger.log(f"Vision error: {e}", level="ERROR", source="Vision")
            time.sleep(0.01)

    logger.log("Vision thread stopped.", source="Vision")


# =============================================================================
# Thread 2: MAVLink Vision Position Transmitter
# =============================================================================

def mavlink_tx_thread(mav, ekf, flight_ctrl, logger):
    """
    Sends VISION_POSITION_ESTIMATE to Pixhawk at the configured rate.

    Also logs telemetry to CSV at each transmission.
    """
    logger.log(f"MAVLink TX thread started ({VISION_SEND_RATE_HZ} Hz).", source="MAV_TX")

    interval = 1.0 / VISION_SEND_RATE_HZ

    while not shutdown_event.is_set():
        try:
            # Get fused position from EKF
            pos = ekf.get_position()
            vel = ekf.get_velocity()

            # Send to Pixhawk
            # Note: VISION_POSITION_ESTIMATE uses NED frame
            # x=North, y=East, z=Down (negative = up)
            mav.send_vision_position_estimate(
                x=pos[0],
                y=pos[1],
                z=pos[2],
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
            )

            # Log telemetry
            mode_num, mode_name = mav.get_flight_mode()
            battery_v, battery_pct = mav.get_battery_status()
            state = flight_ctrl.get_state()
            hover_rem = flight_ctrl.get_hover_remaining()

            logger.log_telemetry(
                state=state.value,
                flight_mode=mode_name,
                x=pos[0], y=pos[1], z=pos[2],
                vx=vel[0], vy=vel[1], vz=vel[2],
                hover_remaining=hover_rem,
                battery_v=battery_v or 0.0,
                battery_pct=battery_pct or 0.0,
            )

            time.sleep(interval)

        except Exception as e:
            logger.log(f"MAVLink TX error: {e}", level="ERROR", source="MAV_TX")
            time.sleep(interval)

    logger.log("MAVLink TX thread stopped.", source="MAV_TX")


# =============================================================================
# Thread 3: Flight Logic State Machine
# =============================================================================

def flight_logic_thread(flight_ctrl, logger):
    """
    Flight state machine loop (~10 Hz).

    Calls flight_ctrl.tick() which manages:
      INIT → WAITING_FOR_LOITER → HOVERING (60s) → LANDING → LANDED
    """
    logger.log("Flight logic thread started.", source="FlightLogic")

    while not shutdown_event.is_set():
        state = flight_ctrl.tick()

        if state == FlightState.LANDED:
            logger.log("Mission completed! Initiating shutdown...", source="FlightLogic")
            time.sleep(2)  # Brief pause for final log flush
            shutdown_event.set()
            break

        if state == FlightState.ERROR:
            logger.log(f"Flight error: {flight_ctrl.get_error()}", 
                       level="ERROR", source="FlightLogic")
            shutdown_event.set()
            break

        time.sleep(0.1)  # ~10 Hz tick rate

    logger.log("Flight logic thread stopped.", source="FlightLogic")


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    """
    Main entry point for the ASCEND autonomous hover system.

    Initializes all hardware interfaces and starts the three
    concurrent processing threads.
    """
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ------------------------------------------------------------------
    # Initialize Logger
    # ------------------------------------------------------------------
    logger = MissionLogger()
    logger.log("=" * 70)
    logger.log("  ASCEND — GPS-Denied Autonomous Hover & Auto-Land System")
    logger.log(f"  Hover Duration: {HOVER_DURATION_SEC} seconds")
    logger.log("=" * 70)

    # ------------------------------------------------------------------
    # Initialize MAVLink
    # ------------------------------------------------------------------
    logger.log("Initializing MAVLink connection to Pixhawk...")
    mav = MAVLinkManager(logger=logger)

    if not mav.connect(timeout=30):
        logger.log("FATAL: Could not connect to Pixhawk. Exiting.", level="CRITICAL")
        logger.close()
        sys.exit(1)

    # Request data streams from Pixhawk
    mav.request_data_stream()

    # ------------------------------------------------------------------
    # Initialize RealSense D455
    # ------------------------------------------------------------------
    logger.log("Initializing Intel RealSense D455...")
    realsense = RealSenseManager(logger=logger)

    if not realsense.start_pipeline():
        logger.log("FATAL: Could not start RealSense pipeline. Exiting.", level="CRITICAL")
        mav.disconnect()
        logger.close()
        sys.exit(1)

    # Get camera intrinsics for VO calibration
    intrinsics = realsense.get_depth_intrinsics()
    if intrinsics:
        logger.log(f"Camera intrinsics: fx={intrinsics['fx']:.1f}, "
                   f"fy={intrinsics['fy']:.1f}, "
                   f"cx={intrinsics['cx']:.1f}, cy={intrinsics['cy']:.1f}")

    # ------------------------------------------------------------------
    # Initialize Processing Modules
    # ------------------------------------------------------------------
    vo = VisualOdometry(logger=logger)
    ekf = EKFFusion(logger=logger)
    flight_ctrl = FlightController(mav, logger=logger)

    logger.log("All systems initialized. Starting mission threads...")
    logger.log("")
    logger.log("  PROCEDURE:")
    logger.log("  1. Arm the drone and take off in STABILIZE mode")
    logger.log("  2. Climb to 3-4 meters altitude")
    logger.log("  3. Switch transmitter to LOITER mode")
    logger.log(f"  4. System will hover for {HOVER_DURATION_SEC}s then auto-land")
    logger.log("")

    # ------------------------------------------------------------------
    # Start Threads
    # ------------------------------------------------------------------
    threads = []

    t_vision = threading.Thread(
        target=vision_thread,
        args=(realsense, vo, ekf, logger),
        daemon=True,
        name="VisionThread",
    )
    threads.append(t_vision)

    t_mavtx = threading.Thread(
        target=mavlink_tx_thread,
        args=(mav, ekf, flight_ctrl, logger),
        daemon=True,
        name="MAVLinkTXThread",
    )
    threads.append(t_mavtx)

    t_flight = threading.Thread(
        target=flight_logic_thread,
        args=(flight_ctrl, logger),
        daemon=True,
        name="FlightLogicThread",
    )
    threads.append(t_flight)

    # Start all threads
    for t in threads:
        t.start()
        logger.log(f"  Started thread: {t.name}")

    # ------------------------------------------------------------------
    # Main loop — wait for shutdown
    # ------------------------------------------------------------------
    logger.log("All threads running. Waiting for mission completion or SIGINT...")

    try:
        while not shutdown_event.is_set():
            shutdown_event.wait(timeout=1.0)
    except KeyboardInterrupt:
        logger.log("KeyboardInterrupt — shutting down...")
        shutdown_event.set()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    logger.log("Shutting down all systems...")

    # Wait for threads to finish
    for t in threads:
        t.join(timeout=5)
        if t.is_alive():
            logger.log(f"  Thread {t.name} did not stop cleanly.", level="WARNING")

    # Cleanup hardware
    realsense.stop()
    mav.disconnect()

    logger.log("=" * 70)
    logger.log("  ASCEND mission shutdown complete.")
    logger.log("=" * 70)

    logger.close()


if __name__ == "__main__":
    main()
