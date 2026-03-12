"""
realsense_manager.py — Intel RealSense D455 Pipeline Manager

Handles:
  - Pipeline initialization (depth, IR/RGB, IMU streams)
  - Frame acquisition
  - IMU data extraction (accelerometer + gyroscope)
  - Graceful shutdown
"""

import time
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None
    print("[RealSense] WARNING: pyrealsense2 not installed. "
          "Install it on the Raspberry Pi with: pip install pyrealsense2")

from src.config import (
    RS_DEPTH_WIDTH, RS_DEPTH_HEIGHT, RS_DEPTH_FPS,
    RS_IR_WIDTH, RS_IR_HEIGHT, RS_IR_FPS,
    RS_ACCEL_FPS, RS_GYRO_FPS,
)


class RealSenseManager:
    """Manages the Intel RealSense D455 camera pipeline."""

    def __init__(self, logger=None):
        """
        Args:
            logger: Optional logger instance.
        """
        if rs is None:
            raise RuntimeError("pyrealsense2 is required but not installed.")

        self.logger = logger
        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None

        # Latest IMU data
        self._latest_accel = np.zeros(3)   # [ax, ay, az] m/s²
        self._latest_gyro = np.zeros(3)    # [gx, gy, gz] rad/s
        self._imu_timestamp = 0.0
        self.has_imu = False
        
        # Hardware info
        self.device_name = "Unknown"
        self.serial_number = "Unknown"
        self.usb_type = "Unknown"

    # -------------------------------------------------------------------------
    # Pipeline Control
    # -------------------------------------------------------------------------

    def start_pipeline(self):
        """
        Initialize and start the RealSense D455 pipeline with:
          - Depth stream
          - Infrared stream (for optical flow)
          - Accelerometer stream
          - Gyroscope stream

        Returns:
            True if pipeline started successfully.
        """
        self._log("Initializing RealSense D455 pipeline...")

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable depth stream
        self._log(f"Enabling Depth: {RS_DEPTH_WIDTH}x{RS_DEPTH_HEIGHT} @ {RS_DEPTH_FPS}fps")
        self.config.enable_stream(
            rs.stream.depth,
            RS_DEPTH_WIDTH, RS_DEPTH_HEIGHT,
            rs.format.z16,
            RS_DEPTH_FPS,
        )

        # Enable infrared stream (left IR camera — best for feature tracking)
        self._log(f"Enabling Infrared: {RS_IR_WIDTH}x{RS_IR_HEIGHT} @ {RS_IR_FPS}fps")
        self.config.enable_stream(
            rs.stream.infrared, 1,  # stream index 1 = left IR
            RS_IR_WIDTH, RS_IR_HEIGHT,
            rs.format.y8,
            RS_IR_FPS,
        )

        # 2. Start pipeline-wrapper to query device capabilities
        try:
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            self.device_name = device.get_info(rs.camera_info.name)
            self.serial_number = device.get_info(rs.camera_info.serial_number)
            self.usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
            
            # Check for Motion Module (IMU)
            self.has_imu = False
            for sensor in device.query_sensors():
                if "Motion Module" in sensor.get_info(rs.camera_info.name):
                    self.has_imu = True
                    break
                    
        except Exception as e:
            self._log(f"Warning: Could not pre-query device: {e}", level="WARNING")

        # 3. Enable IMU streams (ONLY if hardware and kernel support it)
        if self.has_imu:
            self._log(f"Enabling IMU (Accel @ {RS_ACCEL_FPS}fps, Gyro @ {RS_GYRO_FPS}fps)")
            try:
                self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, RS_ACCEL_FPS)
                self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, RS_GYRO_FPS)
            except Exception as e:
                self._log(f"Warning: Failed to enable IMU streams: {e}", level="WARNING")
                self.has_imu = False
        else:
            self._log("IMU (Motion Module) not detected. Continuing with Vision only.", level="WARNING")

        # Start pipeline
        try:
            self.profile = self.pipeline.start(self.config)
            self._log("RealSense pipeline started successfully.")

            # Create alignment object (align depth to IR)
            self.align = rs.align(rs.stream.infrared)

            # Log device info
            device = self.profile.get_device()
            info = device.get_info(rs.camera_info.name)
            usb = device.get_info(rs.camera_info.usb_type_descriptor)
            self._log(f"Device: {info} (USB {usb})")
            
            return True
        except Exception as e:
            self._log(f"Pipeline Start FAILED: {e}", level="ERROR")
            self._log("TIP: This usually means the USB cable cannot handle the data rate or the resolution is unsupported.", level="WARNING")
            return False

    def stop(self):
        """Gracefully stop the RealSense pipeline."""
        if self.pipeline:
            try:
                self.pipeline.stop()
                self._log("RealSense pipeline stopped.")
            except Exception as e:
                self._log(f"Error stopping pipeline: {e}", level="WARNING")

    # -------------------------------------------------------------------------
    # Frame Acquisition
    # -------------------------------------------------------------------------

    def get_sensor_data(self, timeout_ms=2000):
        """
        Wait for and return the latest frameset (Images + IMU).
        
        Using a single wait_for_frames call is critical to avoid
        timeouts on Raspberry Pi.
        
        Returns:
            Tuple (ir_frame_np, depth_frame_np, timestamp) or (None, None, None).
            IMU data is cached internally and can be retrieved via get_imu_data().
        """
        try:
            frameset = self.pipeline.wait_for_frames(timeout_ms)

            # --- 1. Update IMU data (if supported) ---
            if self.has_imu:
                accel_frame = frameset.first_or_default(rs.stream.accel)
                if accel_frame:
                    accel_data = accel_frame.as_motion_frame().get_motion_data()
                    self._latest_accel = np.array([accel_data.x, accel_data.y, accel_data.z])
                    self._imu_timestamp = accel_frame.get_timestamp() / 1000.0

                gyro_frame = frameset.first_or_default(rs.stream.gyro)
                if gyro_frame:
                    gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                    self._latest_gyro = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
            else:
                self._imu_timestamp = time.time()  # Use system time for logic consistency

            # --- 2. Get Video frames ---
            # Get IR frame
            ir_frame = frameset.get_infrared_frame(1)
            # Get depth frame
            depth_frame = frameset.get_depth_frame()

            if not ir_frame or not depth_frame:
                return None, None, None

            # Convert to numpy arrays
            ir_np = np.asanyarray(ir_frame.get_data())
            depth_np = np.asanyarray(depth_frame.get_data())
            timestamp = frameset.get_timestamp() / 1000.0

            return ir_np, depth_np, timestamp

        except Exception as e:
            self._log(f"Sensor acquisition error: {e}", level="ERROR")
            return None, None, None

    def get_imu_data(self):
        """
        Return the latest cached IMU data.
        
        Non-blocking. Data is updated during get_sensor_data() calls.
        """
        return self._latest_accel.copy(), self._latest_gyro.copy(), self._imu_timestamp

    # -------------------------------------------------------------------------
    # Depth Utilities
    # -------------------------------------------------------------------------

    def get_depth_at_pixel(self, depth_frame_np, u, v):
        """
        Get depth in meters at pixel (u, v).

        Args:
            depth_frame_np: Depth frame as numpy array (in mm).
            u, v: Pixel coordinates.

        Returns:
            Depth in meters, or 0.0 if invalid.
        """
        h, w = depth_frame_np.shape
        if 0 <= v < h and 0 <= u < w:
            depth_mm = depth_frame_np[int(v), int(u)]
            return depth_mm / 1000.0  # mm → meters
        return 0.0

    def get_center_depth(self, depth_frame_np):
        """
        Get depth at the center of the depth image (useful for altitude estimation).

        Args:
            depth_frame_np: Depth frame numpy array.

        Returns:
            Depth in meters at image center.
        """
        h, w = depth_frame_np.shape
        return self.get_depth_at_pixel(depth_frame_np, w // 2, h // 2)

    def get_depth_intrinsics(self):
        """
        Get the depth camera intrinsics from the active profile.

        Returns:
            Dictionary with fx, fy, cx, cy, width, height.
        """
        if self.profile is None:
            return None

        depth_stream = self.profile.get_stream(rs.stream.depth)
        intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

        return {
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.ppx,
            "cy": intrinsics.ppy,
            "width": intrinsics.width,
            "height": intrinsics.height,
        }

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="RealSense")
        else:
            print(f"[RealSense] [{level}] {message}")
