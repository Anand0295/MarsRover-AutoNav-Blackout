#!/usr/bin/env python3
"""
Sensor Fusion Module for Mars Rover Autonomous Navigation

This module handles multi-sensor data fusion for robust navigation
during communication blackout periods.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class SensorType(Enum):
    """Enumeration of available sensor types"""
    IMU = "imu"
    LIDAR = "lidar"
    CAMERA = "camera"
    WHEEL_ODOMETRY = "wheel_odometry"
    GPS = "gps"


@dataclass
class SensorReading:
    """Container for sensor data with metadata"""
    sensor_type: SensorType
    timestamp: float
    data: np.ndarray
    confidence: float
    variance: Optional[np.ndarray] = None


class KalmanFilter:
    """Extended Kalman Filter for state estimation"""
    
    def __init__(self, state_dim: int, measurement_dim: int):
        """
        Initialize Kalman Filter
        
        Args:
            state_dim: Dimension of state vector
            measurement_dim: Dimension of measurement vector
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # State vector [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(state_dim)
        
        # State covariance matrix
        self.P = np.eye(state_dim) * 0.1
        
        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.01
        
        # Measurement noise covariance
        self.R = np.eye(measurement_dim) * 0.1
        
    def predict(self, dt: float, control_input: Optional[np.ndarray] = None):
        """Predict next state based on motion model"""
        # Simple constant velocity model
        F = np.eye(self.state_dim)
        if self.state_dim >= 6:
            F[0, 3] = dt  # x += vx * dt
            F[1, 4] = dt  # y += vy * dt
            F[2, 5] = dt  # z += vz * dt
        
        self.state = F @ self.state
        if control_input is not None:
            self.state += control_input
            
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, measurement: np.ndarray, H: np.ndarray):
        """Update state estimate with new measurement"""
        # Innovation
        y = measurement - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P
        
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get current state estimate and covariance"""
        return self.state.copy(), self.P.copy()


class SensorFusion:
    """Main sensor fusion class for multi-modal data integration"""
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize sensor fusion system
        
        Args:
            config: Configuration dictionary with sensor parameters
        """
        self.config = config or self._default_config()
        self.kf = KalmanFilter(state_dim=9, measurement_dim=3)
        self.sensor_buffer: Dict[SensorType, List[SensorReading]] = {}
        self.last_update_time = 0.0
        
    def _default_config(self) -> Dict:
        """Return default configuration"""
        return {
            "buffer_size": 100,
            "fusion_rate": 50.0,  # Hz
            "sensor_weights": {
                SensorType.IMU: 0.3,
                SensorType.LIDAR: 0.4,
                SensorType.CAMERA: 0.2,
                SensorType.WHEEL_ODOMETRY: 0.1
            }
        }
    
    def add_sensor_reading(self, reading: SensorReading):
        """Add new sensor reading to buffer"""
        if reading.sensor_type not in self.sensor_buffer:
            self.sensor_buffer[reading.sensor_type] = []
        
        self.sensor_buffer[reading.sensor_type].append(reading)
        
        # Maintain buffer size
        max_size = self.config["buffer_size"]
        if len(self.sensor_buffer[reading.sensor_type]) > max_size:
            self.sensor_buffer[reading.sensor_type].pop(0)
    
    def fuse_sensors(self, timestamp: float) -> Tuple[np.ndarray, float]:
        """
        Fuse all available sensor data at given timestamp
        
        Args:
            timestamp: Current timestamp
            
        Returns:
            Tuple of (fused_state, confidence)
        """
        dt = timestamp - self.last_update_time
        
        # Prediction step
        self.kf.predict(dt)
        
        # Update with available sensors
        total_confidence = 0.0
        sensor_weights = self.config["sensor_weights"]
        
        for sensor_type, readings in self.sensor_buffer.items():
            if not readings:
                continue
                
            # Get most recent reading
            latest = readings[-1]
            
            if abs(latest.timestamp - timestamp) < 0.1:  # 100ms threshold
                # Construct measurement matrix based on sensor type
                H = self._get_measurement_matrix(sensor_type)
                
                # Update filter
                self.kf.update(latest.data, H)
                
                # Accumulate confidence
                weight = sensor_weights.get(sensor_type, 0.1)
                total_confidence += latest.confidence * weight
        
        self.last_update_time = timestamp
        state, _ = self.kf.get_state()
        
        return state, min(total_confidence, 1.0)
    
    def _get_measurement_matrix(self, sensor_type: SensorType) -> np.ndarray:
        """Get measurement matrix H for specific sensor type"""
        H = np.zeros((3, 9))
        
        if sensor_type == SensorType.LIDAR:
            # LIDAR measures position
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0
        elif sensor_type == SensorType.IMU:
            # IMU measures orientation
            H[0, 6] = 1.0
            H[1, 7] = 1.0
            H[2, 8] = 1.0
        elif sensor_type == SensorType.WHEEL_ODOMETRY:
            # Wheel odometry measures velocity
            H[0, 3] = 1.0
            H[1, 4] = 1.0
            H[2, 5] = 1.0
        else:
            # Default to position measurement
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0
            
        return H
    
    def get_position(self) -> np.ndarray:
        """Get current position estimate [x, y, z]"""
        state, _ = self.kf.get_state()
        return state[:3]
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate [vx, vy, vz]"""
        state, _ = self.kf.get_state()
        return state[3:6]
    
    def get_orientation(self) -> np.ndarray:
        """Get current orientation estimate [roll, pitch, yaw]"""
        state, _ = self.kf.get_state()
        return state[6:9]
    
    def reset(self):
        """Reset fusion system to initial state"""
        self.kf = KalmanFilter(state_dim=9, measurement_dim=3)
        self.sensor_buffer.clear()
        self.last_update_time = 0.0


def main():
    """Example usage of sensor fusion module"""
    # Initialize sensor fusion
    fusion = SensorFusion()
    
    # Simulate sensor readings
    timestamp = 0.0
    
    # Add IMU reading
    imu_reading = SensorReading(
        sensor_type=SensorType.IMU,
        timestamp=timestamp,
        data=np.array([0.0, 0.0, 0.0]),  # roll, pitch, yaw
        confidence=0.95
    )
    fusion.add_sensor_reading(imu_reading)
    
    # Add LIDAR reading
    lidar_reading = SensorReading(
        sensor_type=SensorType.LIDAR,
        timestamp=timestamp,
        data=np.array([1.0, 2.0, 0.5]),  # x, y, z position
        confidence=0.90
    )
    fusion.add_sensor_reading(lidar_reading)
    
    # Perform fusion
    state, confidence = fusion.fuse_sensors(timestamp)
    
    print(f"Fused State: {state}")
    print(f"Confidence: {confidence:.2f}")
    print(f"Position: {fusion.get_position()}")
    print(f"Velocity: {fusion.get_velocity()}")
    print(f"Orientation: {fusion.get_orientation()}")


if __name__ == "__main__":
    main()
