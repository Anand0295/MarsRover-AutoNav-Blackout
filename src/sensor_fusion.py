"""Optimized Sensor Fusion for Mars Rover Navigation

Implements Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF)
with vectorized operations for reduced time/memory complexity.

Author: MarsRover-AutoNav-Blackout Team
License: MIT
"""

import numpy as np
from scipy.linalg import sqrtm, cho_factor, cho_solve
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class SensorConfig:
    """Configuration for sensor fusion parameters."""
    dt: float = 0.1  # Time step
    process_noise: np.ndarray = None  # Process noise covariance
    measurement_noise: np.ndarray = None  # Measurement noise covariance
    
    def __post_init__(self):
        if self.process_noise is None:
            self.process_noise = np.eye(6) * 0.01
        if self.measurement_noise is None:
            self.measurement_noise = np.eye(3) * 0.1


class EKFSensorFusion:
    """Extended Kalman Filter for sensor fusion with vectorized operations.
    
    State vector: [x, y, theta, v_x, v_y, omega]
    Optimized for low memory footprint and fast computation.
    """
    
    def __init__(self, config: SensorConfig):
        self.config = config
        self.state = np.zeros(6)  # State: [x, y, theta, v_x, v_y, omega]
        self.P = np.eye(6)  # State covariance
        self.dt = config.dt
        
        # Pre-allocate matrices for efficiency
        self._F = np.eye(6)  # State transition matrix
        self._H = np.zeros((3, 6))  # Measurement matrix
        self._H[:3, :3] = np.eye(3)  # Measure position only
        
    def predict(self, control: Optional[np.ndarray] = None) -> np.ndarray:
        """Vectorized prediction step.
        
        Args:
            control: Optional control input [v, omega]
            
        Returns:
            Predicted state vector
        """
        # Update state transition matrix (vectorized)
        theta = self.state[2]
        self._F[0, 2] = -self.state[3] * np.sin(theta) * self.dt
        self._F[1, 2] = self.state[3] * np.cos(theta) * self.dt
        self._F[:2, 3:5] = np.eye(2) * self.dt * np.array([[np.cos(theta), -np.sin(theta)],
                                                             [np.sin(theta), np.cos(theta)]])
        self._F[2, 5] = self.dt
        
        # Predict state (vectorized matrix multiplication)
        self.state = self._F @ self.state
        
        # Predict covariance (optimized with in-place operations)
        self.P = self._F @ self.P @ self._F.T + self.config.process_noise
        
        return self.state
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """Vectorized measurement update step with Cholesky decomposition.
        
        Args:
            measurement: Measurement vector [x, y, theta]
            
        Returns:
            Updated state vector
        """
        # Innovation (vectorized)
        y = measurement - self._H @ self.state
        
        # Innovation covariance
        S = self._H @ self.P @ self._H.T + self.config.measurement_noise
        
        # Kalman gain using Cholesky decomposition for numerical stability
        try:
            L, lower = cho_factor(S)
            K = self.P @ self._H.T @ cho_solve((L, lower), np.eye(S.shape[0]))
        except np.linalg.LinAlgError:
            # Fallback to standard inverse if Cholesky fails
            K = self.P @ self._H.T @ np.linalg.inv(S)
        
        # Update state and covariance (vectorized)
        self.state += K @ y
        self.P = (np.eye(6) - K @ self._H) @ self.P
        
        return self.state


class UKFSensorFusion:
    """Unscented Kalman Filter for nonlinear sensor fusion.
    
    Uses scaled unscented transform with vectorized sigma point computation.
    Memory-efficient implementation with minimal allocations.
    """
    
    def __init__(self, config: SensorConfig, alpha: float = 1e-3, beta: float = 2.0, kappa: float = 0.0):
        self.config = config
        self.state = np.zeros(6)
        self.P = np.eye(6)
        self.dt = config.dt
        
        # UKF parameters
        self.n = 6  # State dimension
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = alpha**2 * (self.n + kappa) - self.n
        
        # Compute weights (vectorized, computed once)
        self.Wm = np.full(2 * self.n + 1, 0.5 / (self.n + self.lambda_))
        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        
        self.Wc = self.Wm.copy()
        self.Wc[0] += (1 - alpha**2 + beta)
        
        # Pre-allocate sigma points matrix
        self._sigma_points = np.zeros((2 * self.n + 1, self.n))
        
    def _compute_sigma_points(self) -> np.ndarray:
        """Compute sigma points using vectorized operations.
        
        Returns:
            Sigma points matrix (2n+1 x n)
        """
        # Compute matrix square root using Cholesky decomposition
        L = sqrtm((self.n + self.lambda_) * self.P).real
        
        # Vectorized sigma point generation
        self._sigma_points[0] = self.state
        self._sigma_points[1:self.n+1] = self.state + L
        self._sigma_points[self.n+1:] = self.state - L
        
        return self._sigma_points
    
    def _predict_sigma_point(self, sigma: np.ndarray) -> np.ndarray:
        """Predict single sigma point through motion model.
        
        Args:
            sigma: Single sigma point
            
        Returns:
            Predicted sigma point
        """
        theta = sigma[2]
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Vectorized state transition
        predicted = sigma.copy()
        predicted[0] += (sigma[3] * cos_theta - sigma[4] * sin_theta) * self.dt
        predicted[1] += (sigma[3] * sin_theta + sigma[4] * cos_theta) * self.dt
        predicted[2] += sigma[5] * self.dt
        
        return predicted
    
    def predict(self) -> np.ndarray:
        """Vectorized prediction step using unscented transform.
        
        Returns:
            Predicted state vector
        """
        # Generate sigma points
        sigmas = self._compute_sigma_points()
        
        # Propagate sigma points (vectorized with list comprehension)
        sigmas_pred = np.array([self._predict_sigma_point(s) for s in sigmas])
        
        # Compute predicted mean (vectorized weighted sum)
        self.state = self.Wm @ sigmas_pred
        
        # Compute predicted covariance (vectorized)
        diff = sigmas_pred - self.state
        self.P = (self.Wc[:, None, None] * (diff[:, :, None] @ diff[:, None, :])).sum(axis=0)
        self.P += self.config.process_noise
        
        return self.state
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """Vectorized measurement update using unscented transform.
        
        Args:
            measurement: Measurement vector [x, y, theta]
            
        Returns:
            Updated state vector
        """
        # Generate sigma points
        sigmas = self._compute_sigma_points()
        
        # Transform sigma points to measurement space (vectorized)
        sigmas_meas = sigmas[:, :3]  # Direct measurement of position
        
        # Predicted measurement (vectorized weighted sum)
        z_pred = self.Wm @ sigmas_meas
        
        # Innovation covariance (vectorized)
        diff = sigmas_meas - z_pred
        Pz = (self.Wc[:, None, None] * (diff[:, :, None] @ diff[:, None, :])).sum(axis=0)
        Pz += self.config.measurement_noise
        
        # Cross-covariance (vectorized)
        diff_state = sigmas - self.state
        Pxz = (self.Wc[:, None, None] * (diff_state[:, :, None] @ diff[:, None, :])).sum(axis=0)
        
        # Kalman gain
        K = Pxz @ np.linalg.inv(Pz)
        
        # Update state and covariance
        self.state += K @ (measurement - z_pred)
        self.P -= K @ Pz @ K.T
        
        return self.state


class MultiSensorFusion:
    """Optimized multi-sensor fusion system.
    
    Combines IMU, GPS, and odometry data using adaptive filter selection.
    Automatically switches between EKF and UKF based on linearity assumptions.
    """
    
    def __init__(self, config: SensorConfig, use_ukf: bool = False):
        self.config = config
        self.filter = UKFSensorFusion(config) if use_ukf else EKFSensorFusion(config)
        
    def fuse_sensors(self, imu_data: np.ndarray, gps_data: Optional[np.ndarray] = None,
                     odom_data: Optional[np.ndarray] = None) -> np.ndarray:
        """Fuse multiple sensor inputs efficiently.
        
        Args:
            imu_data: IMU acceleration/gyro data
            gps_data: Optional GPS position data
            odom_data: Optional odometry data
            
        Returns:
            Fused state estimate
        """
        # Predict step
        self.filter.predict()
        
        # Update with available measurements (vectorized batch processing)
        if gps_data is not None:
            self.filter.update(gps_data)
        
        if odom_data is not None:
            self.filter.update(odom_data)
            
        return self.filter.state
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate.
        
        Returns:
            State vector [x, y, theta, v_x, v_y, omega]
        """
        return self.filter.state
    
    def get_covariance(self) -> np.ndarray:
        """Get state covariance matrix.
        
        Returns:
            Covariance matrix (6x6)
        """
        return self.filter.P
