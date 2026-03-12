"""
ekf_fusion.py — Extended Kalman Filter for Sensor Fusion

Fuses visual odometry position estimates with D455 IMU accelerometer
data to produce a smooth, drift-corrected position estimate.

State vector: [x, y, z, vx, vy, vz]  (position + velocity)
Prediction:   IMU accelerometer (high frequency)
Update:       Visual odometry position (lower frequency, more accurate)
"""

import numpy as np

from src.config import (
    EKF_PROCESS_NOISE_POS,
    EKF_PROCESS_NOISE_VEL,
    EKF_MEASUREMENT_NOISE_VO,
    EKF_INITIAL_COVARIANCE,
)


class EKFFusion:
    """
    6-state Extended Kalman Filter for 3D position estimation.

    State: [x, y, z, vx, vy, vz]
    - Prediction step uses IMU accelerometer data
    - Update step uses visual odometry position measurements
    """

    def __init__(self, logger=None):
        """
        Initialize the EKF with zero state and default covariances.

        Args:
            logger: Optional logger instance.
        """
        self.logger = logger

        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)

        # State covariance matrix (6×6)
        self.P = np.eye(6) * EKF_INITIAL_COVARIANCE

        # Process noise covariance Q (6×6) — set in _build_Q()
        self._q_pos = EKF_PROCESS_NOISE_POS
        self._q_vel = EKF_PROCESS_NOISE_VEL

        # Measurement noise covariance R (3×3) — for position observations
        self.R = np.eye(3) * EKF_MEASUREMENT_NOISE_VO

        # Measurement matrix H: we observe [x, y, z] from visual odometry
        # H maps state [x, y, z, vx, vy, vz] → [x, y, z]
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0  # observe x
        self.H[1, 1] = 1.0  # observe y
        self.H[2, 2] = 1.0  # observe z

        # Gravity vector (m/s²) — D455 IMU reports accel including gravity
        self._gravity = np.array([0.0, 0.0, 9.81])

        # Timestamp for dt calculation
        self._last_time = None

        self._log("EKF initialized with 6-state model [x, y, z, vx, vy, vz]")

    # -------------------------------------------------------------------------
    # Prediction Step (IMU accelerometer)
    # -------------------------------------------------------------------------

    def predict(self, accel, dt):
        """
        EKF prediction step using IMU accelerometer data.

        Propagates the state forward using constant-acceleration kinematics:
            x_new = x + vx * dt + 0.5 * ax * dt²
            vx_new = vx + ax * dt

        Args:
            accel: numpy array [ax, ay, az] in m/s² (raw from D455 IMU).
            dt: Time step in seconds since last prediction.
        """
        if dt <= 0 or dt > 1.0:
            return  # Skip invalid time steps

        # Prepare acceleration input
        if accel is not None and not np.all(accel == 0):
            # Remove gravity from accelerometer reading
            # D455 IMU Z-axis typically reads ~9.81 when stationary
            accel_corrected = accel - self._gravity
        else:
            # Constant velocity model (zero acceleration)
            accel_corrected = np.zeros(3)

        # State transition matrix F (6×6)
        F = np.eye(6)
        F[0, 3] = dt   # x += vx * dt
        F[1, 4] = dt   # y += vy * dt
        F[2, 5] = dt   # z += vz * dt

        # Control input matrix B (6×3)
        B = np.zeros((6, 3))
        B[0, 0] = 0.5 * dt * dt  # x += 0.5 * ax * dt²
        B[1, 1] = 0.5 * dt * dt
        B[2, 2] = 0.5 * dt * dt
        B[3, 0] = dt              # vx += ax * dt
        B[4, 1] = dt
        B[5, 2] = dt

        # Predict state
        self.state = F @ self.state + B @ accel_corrected

        # Process noise Q (scales with dt)
        Q = self._build_Q(dt)

        # Predict covariance
        self.P = F @ self.P @ F.T + Q

    # -------------------------------------------------------------------------
    # Update Step (Visual Odometry)
    # -------------------------------------------------------------------------

    def update(self, vo_position):
        """
        EKF update (correction) step using visual odometry position.

        Corrects the predicted state using the measured position from
        the camera-based visual odometry system.

        Args:
            vo_position: numpy array [x, y, z] measured position in meters.
        """
        z = np.array(vo_position[:3])

        # Innovation (measurement residual)
        y = z - self.H @ self.state

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self._log("WARNING: Singular matrix in Kalman gain computation",
                       level="WARNING")
            return

        # Update state
        self.state = self.state + K @ y

        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    # -------------------------------------------------------------------------
    # Output
    # -------------------------------------------------------------------------

    def get_position(self):
        """
        Get the current fused position estimate.

        Returns:
            numpy array [x, y, z] in meters.
        """
        return self.state[:3].copy()

    def get_velocity(self):
        """
        Get the current fused velocity estimate.

        Returns:
            numpy array [vx, vy, vz] in m/s.
        """
        return self.state[3:6].copy()

    def get_full_state(self):
        """
        Get the complete state vector.

        Returns:
            numpy array [x, y, z, vx, vy, vz].
        """
        return self.state.copy()

    def get_uncertainty(self):
        """
        Get the position uncertainty (standard deviation).

        Returns:
            numpy array [sigma_x, sigma_y, sigma_z] in meters.
        """
        return np.sqrt(np.diag(self.P)[:3])

    # -------------------------------------------------------------------------
    # Reset
    # -------------------------------------------------------------------------

    def reset(self):
        """Reset the EKF to initial state."""
        self.state = np.zeros(6)
        self.P = np.eye(6) * EKF_INITIAL_COVARIANCE
        self._last_time = None
        self._log("EKF reset to origin.")

    def set_position(self, x, y, z):
        """
        Override the current position estimate (e.g., for initialization).

        Args:
            x, y, z: Position in meters.
        """
        self.state[0] = x
        self.state[1] = y
        self.state[2] = z

    # -------------------------------------------------------------------------
    # Internal
    # -------------------------------------------------------------------------

    def _build_Q(self, dt):
        """
        Build the process noise covariance matrix Q.

        Uses a piecewise white noise model where position noise
        scales with dt² and velocity noise scales with dt.

        Args:
            dt: Time step in seconds.

        Returns:
            6×6 process noise matrix.
        """
        Q = np.zeros((6, 6))

        # Position noise
        Q[0, 0] = self._q_pos * dt ** 2
        Q[1, 1] = self._q_pos * dt ** 2
        Q[2, 2] = self._q_pos * dt ** 2

        # Velocity noise
        Q[3, 3] = self._q_vel * dt
        Q[4, 4] = self._q_vel * dt
        Q[5, 5] = self._q_vel * dt

        # Cross-correlation terms
        Q[0, 3] = Q[3, 0] = self._q_pos * dt ** 2 * 0.5
        Q[1, 4] = Q[4, 1] = self._q_pos * dt ** 2 * 0.5
        Q[2, 5] = Q[5, 2] = self._q_pos * dt ** 2 * 0.5

        return Q

    def _log(self, message, level="INFO"):
        """Log through the logger or print."""
        if self.logger:
            self.logger.log(message, level=level, source="EKF")
        else:
            print(f"[EKF] [{level}] {message}")
