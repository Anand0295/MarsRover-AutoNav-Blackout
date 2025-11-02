#!/usr/bin/env python3
"""
Entry point for MarsRover-AutoNav-Blackout demo application.
"""
import time
from sensor_fusion import SensorFusion, SensorReading, SensorType
from blackout_nav import BlackoutNavigator
import numpy as np


def run_demo():
    fusion = SensorFusion()
    nav = BlackoutNavigator(fusion)

    t = 0.0
    dt = 0.1
    for i in range(10):
        # Fake sensors
        fusion.add_sensor_reading(SensorReading(SensorType.IMU, t, np.array([0.0, 0.0, 0.02*i]), 0.9))
        fusion.add_sensor_reading(SensorReading(SensorType.LIDAR, t, np.array([0.1*i, 0.0, 0.0]), 0.85))
        cmd, safety = nav.plan_step(t, clearance_ahead=1.5)
        print(f"t={t:.1f} cmd={cmd} safety={safety:.2f}")
        t += dt


if __name__ == "__main__":
    run_demo()
