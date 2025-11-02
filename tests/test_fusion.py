import numpy as np
from src.sensor_fusion import SensorFusion, SensorReading, SensorType

def test_fuse_basic():
    f = SensorFusion()
    t = 0.0
    f.add_sensor_reading(SensorReading(SensorType.IMU, t, np.array([0,0,0]), 0.9))
    f.add_sensor_reading(SensorReading(SensorType.LIDAR, t, np.array([1,2,0.5]), 0.9))
    state, conf = f.fuse_sensors(t)
    assert state.shape[0] == 9
    assert conf > 0.1
