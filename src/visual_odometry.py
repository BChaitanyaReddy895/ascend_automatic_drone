"""
visual_odometry.py — Optical Flow Visual Odometry

Computes local position displacement (dx, dy, dz) from consecutive
IR camera frames using Lucas-Kanade sparse optical flow and depth data.

Pipeline:
  IR Frame → Detect features → Track across frames → Use depth to scale → Output (dx, dy, dz)
"""

import cv2
import numpy as np

from src.config import (
    LK_WIN_SIZE,
    LK_MAX_LEVEL,
    LK_CRITERIA,
    FEATURE_MAX_CORNERS,
    FEATURE_QUALITY_LEVEL,
    FEATURE_MIN_DISTANCE,
    FEATURE_BLOCK_SIZE,
    VO_USE_CLAHE,
    VO_CLAHE_CLIP,
    VO_CLAHE_GRID,
    CAMERA_FX,
    CAMERA_FY,
    CAMERA_CX,
    CAMERA_CY,
    MIN_FEATURES_THRESHOLD,
    VO_MIN_ALTITUDE,
    VO_MAX_ALTITUDE,
    VO_XY_SCALE,
    VO_Z_SCALE,
    VO_INVERT_X,
    VO_INVERT_Y,
    VO_SWAP_XY,
)


class VisualOdometry:
    """
    Sparse optical flow visual odometry using Lucas-Kanade tracker.

    Computes incremental 3D motion (dx, dy, dz) between consecutive
    grayscale frames using tracked features and depth information.
    """

    def __init__(self, logger=None):
        """
        Args:
            logger: Optional logger instance.
        """
        self.logger = logger

        # Previous frame and tracked points
        self._prev_frame = None
        self._prev_points = None

        # Accumulated position
        self.position = np.zeros(3)  # [x, y, z] in meters

        # OpenCV Lucas-Kanade parameters
        criteria_type = cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT
        self._lk_params = {
            "winSize": LK_WIN_SIZE,
            "maxLevel": LK_MAX_LEVEL,
            "criteria": (
                criteria_type,
                LK_CRITERIA["max_iter"],
                LK_CRITERIA["epsilon"],
            ),
        }

        # Shi-Tomasi corner detection parameters
        self._feature_params = {
            "maxCorners": FEATURE_MAX_CORNERS,
            "qualityLevel": FEATURE_QUALITY_LEVEL,
            "minDistance": FEATURE_MIN_DISTANCE,
            "blockSize": FEATURE_BLOCK_SIZE,
        }

        # Camera intrinsics
        self._fx = CAMERA_FX
        self._fy = CAMERA_FY
        self._cx = CAMERA_CX
        self._cy = CAMERA_CY

        # CLAHE for contrast enhancement
        self._clahe = cv2.createCLAHE(clipLimit=VO_CLAHE_CLIP, tileGridSize=VO_CLAHE_GRID)

        # Frame counter
        self._frame_count = 0

    # -------------------------------------------------------------------------
    # Main Update
    # -------------------------------------------------------------------------

    def update(self, ir_frame_raw, depth_frame):
        """
        Process a new frame pair and compute incremental motion.
        """
        self._frame_count += 1
        
        # 0. Pre-process IR frame for better contrast
        if VO_USE_CLAHE:
            ir_frame = self._clahe.apply(ir_frame_raw)
        else:
            ir_frame = ir_frame_raw.copy()

        # First frame — just detect features
        if self._prev_frame is None:
            self._prev_frame = ir_frame.copy()
            self._prev_points = self._detect_features(ir_frame)
            self._log(f"Initial features detected: {len(self._prev_points) if self._prev_points is not None else 0}")
            return 0.0, 0.0, 0.0

        # Need features to track
        if self._prev_points is None or len(self._prev_points) < MIN_FEATURES_THRESHOLD:
            # Create a mask to avoid detects on reflections
            mask = self._create_reflection_mask(ir_frame)
            self._prev_points = self._detect_features(ir_frame, mask=mask)
            
            if self._prev_points is None or len(self._prev_points) < 5:
                self._prev_frame = ir_frame.copy()
                return 0.0, 0.0, 0.0

        # Track features using Lucas-Kanade optical flow
        curr_points, status, err = cv2.calcOpticalFlowPyrLK(
            self._prev_frame, ir_frame,
            self._prev_points, None,
            **self._lk_params,
        )

        if curr_points is None or status is None:
            self._prev_frame = ir_frame.copy()
            self._prev_points = self._detect_features(ir_frame)
            return 0.0, 0.0, 0.0

        # Filter: keep only successfully tracked points
        good_mask = status.flatten() == 1
        good_prev = self._prev_points[good_mask]
        good_curr = curr_points[good_mask]

        if len(good_prev) < 5:
            self._log(f"Tracking LOST (only {len(good_prev)} points). Re-detecting...", level="WARNING")
            self._prev_frame = ir_frame.copy()
            self._prev_points = self._detect_features(ir_frame)
            return 0.0, 0.0, 0.0

        # 1. Estimate current altitude from depth center
        h, w = depth_frame.shape
        center_d_mm = depth_frame[h // 2, w // 2]
        current_alt_m = center_d_mm / 1000.0 if center_d_mm > 0 else VO_MIN_ALTITUDE
        
        if self._frame_count % 30 == 0:
            avg_dx = np.mean(good_curr[:, 0, 0] - good_prev[:, 0, 0]) if len(good_prev) > 0 else 0
            avg_dy = np.mean(good_curr[:, 0, 1] - good_prev[:, 0, 1]) if len(good_prev) > 0 else 0
            # Calculate average brightness to check if IR laser is working
            avg_bright = np.mean(ir_frame)
            self._log(f"VO Status: features={len(good_prev)}, alt={current_alt_m:.2f}m, flow=({avg_dx:.2f}, {avg_dy:.2f}), IR_bright={avg_bright:.1f}")
            
            if avg_bright < 10:
                self._log("CRITICAL: IR Image is too dark! Tracking will FAIL. Check IR Laser.", level="ERROR")
        
        # 2. Compute 3D incremental motion (dx, dy, dz)
        # We use Global Altitude for XY scaling (stable) 
        # but we use feature-delta-median for Z as requested.
        dx_cam, dy_cam, dz_cam = self._compute_delta_motion(
            good_prev, good_curr, depth_frame, current_alt_m
        )
        
        # 3. Handle Altitude (absolute for display, incremental for pos)
        if not hasattr(self, '_prev_alt_m'):
            self._prev_alt_m = current_alt_m
        self._prev_alt_m = current_alt_m

        # 4. Global Scaling & NED Rotation
        # 4.1 Apply simple scaling
        dx_val = dx_cam * VO_XY_SCALE  # Image DOWN -> Drone FORWARD (+X)
        dy_val = dy_cam * VO_XY_SCALE # Image RIGHT -> Drone LEFT (-Y)
        dz_val = -dz_cam * VO_Z_SCALE   # Alt Increase -> Z Decrease (-Z)
        
        # 4.2 Apply calibration flags from config.py
        if VO_SWAP_XY:
            dx_val, dy_val = dy_val, dx_val
        
        if VO_INVERT_X:
            dx_val = -dx_val
            
        if VO_INVERT_Y:
            dy_val = -dy_val

        # Final NED values
        dx = dx_val
        dy = dy_val
        dz = dz_val

        # Update accumulated position
        self.position[0] += dx
        self.position[1] += dy
        self.position[2] += dz

        # Update state for next frame
        self._prev_frame = ir_frame.copy()
        self._prev_points = good_curr.reshape(-1, 1, 2)

        # Re-detect features if too few remain
        if len(good_curr) < MIN_FEATURES_THRESHOLD:
            mask = self._create_reflection_mask(ir_frame)
            new_points = self._detect_features(ir_frame, mask=mask)
            if new_points is not None:
                self._prev_points = new_points

        return dx, dy, dz

    def get_position(self):
        """
        Get the current accumulated position estimate.

        Returns:
            numpy array [x, y, z] in meters.
        """
        return self.position.copy()

    def reset(self):
        """Reset odometry to origin."""
        self.position = np.zeros(3)
        self._prev_frame = None
        self._prev_points = None
        self._frame_count = 0
        self._log("Visual odometry reset to origin.")

    # -------------------------------------------------------------------------
    # Feature Detection
    # -------------------------------------------------------------------------

    def _create_reflection_mask(self, frame):
        """Create a mask to ignore extremely bright reflections and image edges (like drone landing gear)."""
        _, mask = cv2.threshold(frame, 240, 255, cv2.THRESH_BINARY_INV)
        
        # Mask out outer 15% to avoid tracking drone's own legs/frame
        h, w = frame.shape
        margin_h = int(h * 0.15)
        margin_w = int(w * 0.15)
        mask[0:margin_h, :] = 0
        mask[h-margin_h:, :] = 0
        mask[:, 0:margin_w] = 0
        mask[:, w-margin_w:] = 0
        
        return mask

    def _detect_features(self, frame, mask=None):
        """Detect corner features in a grayscale frame."""
        return cv2.goodFeaturesToTrack(frame, mask=mask, **self._feature_params)

    # -------------------------------------------------------------------------
    # Metric Motion Computation
    # -------------------------------------------------------------------------

    def _compute_delta_motion(self, prev_pts, curr_pts, depth_frame, altitude_m):
        """
        Compute metric displacement by combining optical flow and depth.
        """
        dx_list = []
        dy_list = []
        dz_list = []
        
        h, w = depth_frame.shape
        
        # Constants for Z-check
        depth_min_mm = int(VO_MIN_ALTITUDE * 1000)
        depth_max_mm = int(VO_MAX_ALTITUDE * 1000)

        for i in range(len(prev_pts)):
            u_prev = prev_pts[i].flatten()
            u_curr = curr_pts[i].flatten()
            
            # Pixel Flow
            du = u_curr[0] - u_prev[0]
            dv = u_curr[1] - u_prev[1]
            
            # 1. XY Motion: Scale by global altitude for stability
            if altitude_m > 0.1:
                dx_list.append((du / self._fx) * altitude_m)
                dy_list.append((dv / self._fy) * altitude_m)
            
            # 2. Z Motion: Use incremental feature-depth change (user preference)
            px, py = int(u_prev[0]), int(u_prev[1])
            cx, cy = int(u_curr[0]), int(u_curr[1])
            
            if 0 <= px < w and 0 <= py < h and 0 <= cx < w and 0 <= cy < h:
                d_prev = int(depth_frame[py, px])
                d_curr = int(depth_frame[cy, cx])
                
                if depth_min_mm < d_prev < depth_max_mm and depth_min_mm < d_curr < depth_max_mm:
                    dz_list.append((d_curr - d_prev) / 1000.0)

        dx = float(np.median(dx_list)) if dx_list else 0.0
        dy = float(np.median(dy_list)) if dy_list else 0.0
        dz = float(np.median(dz_list)) if dz_list else 0.0
        
        return dx, dy, dz

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="VisualOdom")
        else:
            print(f"[VisualOdom] [{level}] {message}")
