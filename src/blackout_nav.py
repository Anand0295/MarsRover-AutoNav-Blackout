#!/usr/bin/env python3
"""
Blackout Navigation Module

Provides autonomous navigation behaviors and fallback strategies when the rover
is in a communications blackout. Integrates with SensorFusion to maintain
robust state estimation and executes conservative motion plans and safety checks.
"""
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from .sensor_fusion import SensorFusion, SensorReading, SensorType


@dataclass
class NavConfig:
    max_speed: float = 0.3  # m/s
    min_clearance: float = 0.4  # meters
    reorientation_interval: float = 5.0  # seconds
    breadcrumb_interval: float = 10.0  # seconds
    stop_on_low_confidence: float = 0.35


class BlackoutNavigator:
    """Blackout-safe navigation policy"""

    def __init__(self, fusion: Optional[SensorFusion] = None, config: Optional[NavConfig] = None):
        self.fusion = fusion or SensorFusion()
        self.cfg = config or NavConfig()
        self._last_reorient = 0.0
        self._last_breadcrumb = 0.0

    def plan_step(self, timestamp: float, clearance_ahead: float) -> Tuple[np.ndarray, float]:
        """
        Compute a velocity command during blackout.
        Returns (vx, vy, wz) and a safety score [0..1].
        """
        state, conf = self.fusion.fuse_sensors(timestamp)
        pos = state[:3]
        vel = state[3:6]
        yaw = state[8]

        # Safety gating
        if conf < self.cfg.stop_on_low_confidence:
            return np.array([0.0, 0.0, 0.0]), 0.2
        if clearance_ahead < self.cfg.min_clearance:
            # Stop and rotate in place to scan
            return np.array([0.0, 0.0, 0.3]), 0.4

        # Nominal forward motion in heading frame
        speed = min(self.cfg.max_speed, max(0.05, np.linalg.norm(vel[:2]) + 0.05))
        vx = speed * np.cos(yaw)
        vy = speed * np.sin(yaw)
        wz = 0.0

        # Periodic gentle reorientation to improve perception parallax
        if timestamp - self._last_reorient > self.cfg.reorientation_interval:
            wz = 0.15
            self._last_reorient = timestamp

        # Breadcrumb placeholder: could drop visual markers or save local submap keyframe
        if timestamp - self._last_breadcrumb > self.cfg.breadcrumb_interval:
            self._last_breadcrumb = timestamp
            # no-op placeholder

        safety = min(1.0, 0.6 + 0.4 * conf)
        return np.array([vx, vy, wz]), safety


def demo():
    fusion = SensorFusion()
    nav = BlackoutNavigator(fusion)

    t = 0.0
    # Seed with a yaw estimate and a lidar pose
    fusion.add_sensor_reading(SensorReading(SensorType.IMU, t, np.array([0.0, 0.0, 0.0]), 0.9))
    fusion.add_sensor_reading(SensorReading(SensorType.LIDAR, t, np.array([0.0, 0.0, 0.0]), 0.9))

    cmd, safety = nav.plan_step(t, clearance_ahead=1.0)
    print("cmd:", cmd, "safety:", safety)


if __name__ == "__main__":
    demo()
