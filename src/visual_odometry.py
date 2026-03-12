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
    MIN_FEATURES_THRESHOLD,
    CAMERA_FX,
    CAMERA_FY,
    CAMERA_CX,
    CAMERA_CY,
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

        # Frame counter
        self._frame_count = 0

    # -------------------------------------------------------------------------
    # Main Update
    # -------------------------------------------------------------------------

    def update(self, ir_frame, depth_frame):
        """
        Process a new frame pair and compute incremental motion.

        Args:
            ir_frame: Grayscale IR image (numpy array, H×W, uint8).
            depth_frame: Depth image (numpy array, H×W, uint16 in mm).

        Returns:
            Tuple (dx, dy, dz) incremental displacement in meters,
            or (0, 0, 0) if tracking not possible yet.
        """
        self._frame_count += 1

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
            self._prev_frame = ir_frame.copy()
            self._prev_points = self._detect_features(ir_frame)
            return 0.0, 0.0, 0.0

        # Compute 3D motion from 2D pixel displacements using a Global Altitude model
        # 1. Get stable global altitude from the median depth of the frame
        h, w = depth_frame.shape
        center_crop = depth_frame[h//4:3*h//4, w//4:3*w//4]
        valid_depths = center_crop[center_crop > 0]
        
        if valid_depths.size > 0:
            current_alt_m = np.median(valid_depths) / 1000.0
        else:
            current_alt_m = 0.0

        # 2. Compute XY displacements scaled by this altitude
        dx_cam, dy_cam = self._compute_xy_displacement(
            good_prev, good_curr, current_alt_m
        )

        # 3. Handle Z (Altitude change)
        if not hasattr(self, '_prev_alt_m'):
            self._prev_alt_m = current_alt_m
        
        dz_cam = current_alt_m - self._prev_alt_m
        self._prev_alt_m = current_alt_m

        # 4. Correct Coordinate Frame (Camera -> Drone Body NED)
        # Assuming camera top faces Drone Front and points DOWN:
        # Features move DOWN (+V) -> Drone moves FORWARD (+X)
        # Features move LEFT (-U) -> Drone moves RIGHT (+Y)
        dx = dy_cam    # Forward movement
        dy = -dx_cam   # Right movement
        dz = -dz_cam   # Altitude increase (pos dz_cam) -> Z becomes more negative (NED)

        # Filter sudden jumps (outlier rejection)
        if abs(dx) > 1.0 or abs(dy) > 1.0 or abs(dz) > 0.5:
            return 0.0, 0.0, 0.0

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
        """Create a mask to ignore extremely bright reflections."""
        _, mask = cv2.threshold(frame, 240, 255, cv2.THRESH_BINARY_INV)
        return mask

    def _detect_features(self, frame, mask=None):
        """Detect corner features in a grayscale frame."""
        return cv2.goodFeaturesToTrack(frame, mask=mask, **self._feature_params)

    # -------------------------------------------------------------------------
    # Metric Motion Computation
    # -------------------------------------------------------------------------

    def _compute_xy_displacement(self, prev_pts, curr_pts, altitude_m):
        """
        Convert average 2D pixel flow to metric XY movement.
        """
        if altitude_m <= 0.1:  # Too close to ground or invalid
            return 0.0, 0.0

        # Calculate pixel displacements
        du_list = curr_pts[:, 0] - prev_pts[:, 0]
        dv_list = curr_pts[:, 1] - prev_pts[:, 1]

        # Median pixel flow for robustness
        du_median = np.median(du_list)
        dv_median = np.median(dv_list)

        # Scale by altitude and focal length
        # metric = (pixel / focal_length) * depth
        dx_cam = (du_median / self._fx) * altitude_m
        dy_cam = (dv_median / self._fy) * altitude_m

        return dx_cam, dy_cam

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="VisualOdom")
        else:
            print(f"[VisualOdom] [{level}] {message}")
