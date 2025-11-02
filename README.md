# MarsRover-AutoNav-Blackout

Autonomous Navigation for Mars Rover in Blackout Zones Using Sensor Fusion

Badges: NASA | Mars Society | Python | ROS | OpenCV

---
## Project Overview
This project provides blackout-resilient autonomous navigation with robust sensor fusion and low-cost path planning that continue operating without GPS/comm links.

---
## Technical Highlights (OSS-optimized)
- Sensor Fusion: EKF/UKF with vectorized routines (NumPy/Scipy) for low time/memory cost
- Path Planning: Vectorized A* on occupancy grids, lightweight D* Lite for replanning, optional smoothing
- Containerized Simulation: Dockerfile for ROS Noetic + Gazebo classic

## Repo Structure
- src/sensor_fusion.py — EKF/UKF fusion with vectorized ops and adaptive multi-sensor fusion
- src/path_planning.py — Vectorized A* and minimal D* Lite skeleton with smoothing utilities
- Dockerfile — ROS Noetic + Gazebo base, catkin workspace bootstrap

---
## Quickstart (Open Source Deployment)

### 1) Prerequisites
- Docker (Linux/macOS/Windows with WSL2)
- X11 configured if you want Gazebo GUI (Linux). For macOS/Windows use headless or X server.

### 2) Build Image
```
docker build -t autonov:oss .
```

### 3) Run Container
GUI (Linux X11):
```
xhost +local:root
sudo docker run --rm -it \
  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  autonov:oss bash
```
Headless:
```
sudo docker run --rm -it autonov:oss bash
```

### 4) Use the Library Code
Inside container or host python environment:
```python
import numpy as np
from src.sensor_fusion import SensorConfig, MultiSensorFusion
from src.path_planning import PlannerConfig, AStarPlanner, smooth_path

# Sensor fusion
cfg = SensorConfig(dt=0.1)
fuser = MultiSensorFusion(cfg, use_ukf=True)
imu = np.zeros(6)
gps = np.array([0.0, 0.0, 0.0])
state = fuser.fuse_sensors(imu_data=imu, gps_data=gps)
print("Fused state:", state)

# Path planning
occ = np.zeros((100, 100), dtype=np.uint8)
occ[40:60, 45:55] = 1  # obstacle block
pcfg = PlannerConfig(resolution=0.1, robot_radius=0.3, diag_motion=True)
planner = AStarPlanner(occ, pcfg)
path = planner.plan((10, 10), (80, 80))
print("Raw path length:", len(path) if path else None)
path_s = smooth_path(path)
```

---
## ROS/Gazebo Notes
- Base image: ros:noetic-ros-core plus desktop-full for Gazebo
- catkin workspace at /ws; repository copied to /ws/src/MarsRover-AutoNav-Blackout
- catkin_make is run; Python modules are usable directly

---
## Performance Notes
- EKF update uses Cholesky-based solve; UKF uses vectorized sigma-point ops
- A* uses vectorized neighbor filtering and relaxations to cut Python loops

---
## License
MIT — see LICENSE
