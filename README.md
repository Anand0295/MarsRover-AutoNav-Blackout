# MarsRover-AutoNav-Blackout

# Autonomous Navigation for Mars Rover in Blackout Zones Using Sensor Fusion

[![NASA](https://img.shields.io/badge/NASA-0A0A0A?style=for-the-badge&logo=nasa&logoColor=white)](https://www.nasa.gov) 
[![Mars Society](https://img.shields.io/badge/Mars_Society-FF4500?style=for-the-badge&logo=mars&logoColor=white)](https://www.marssociety.org) 
[![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org) 
[![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://isocpp.org) 
[![ROS](https://img.shields.io/badge/ROS-339933?style=for-the-badge&logo=ros&logoColor=white)](https://www.ros.org) 
[![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org)

---

## Project Overview

The exploration of Mars presents unique challenges for autonomous robotic systems, particularly the frequent loss of GPS or communication signals caused by environmental conditions or planetary obstructions. This project focuses on designing and implementing an advanced autonomous navigation system for a Mars rover capable of safely navigating rough and unstructured terrain, even during extended GPS or communication blackouts.

Unlike traditional rovers that require constant human supervision to proceed, this system allows the rover to make intelligent decisions and continue traversing the terrain independently during blackout periods. Utilizing onboard sensor fusion and sophisticated path-planning algorithms, the rover continuously maintains its trajectory to a pre-planned destination, adapting dynamically to the surrounding environment.

---

## Problem Statement

Mars rovers often face signal disruptions that halt public communication channels and GPS navigation services. These blackouts force rovers to stop moving until they regain connectivity, limiting mission progress and operational efficiency. The inability to move autonomously during these blackout phases creates mission downtime and increases risk.

The objective of this project is to equip a Mars rover with blackout-resilient autonomous navigation capabilities. It enables the rover to:

- Maintain accurate localization using internal sensors without GPS.
- Identify and avoid natural obstacles such as holes, rocks, slopes, and sand pits.
- Dynamically update the path based on real-time terrain and obstacle feedback.
- Navigate with human-like decision-making agility to prevent tripping or falling.
- Complete the journey to the target destination safely and efficiently without external commands.

---

## Technical Approach

### Sensor Fusion for Robust Localization

To achieve continuous localization, we fuse data from multiple sensors onboard:

- **IMU (Inertial Measurement Unit):** Measures acceleration and rotational rates for dead reckoning.
- **LIDAR:** Provides 3D environmental mapping and obstacle detection.
- **Stereo Cameras:** Generate depth maps and terrain texture information.
- **Wheel Encoders and Odometry:** Track wheel rotations for incremental position estimation.

The sensor fusion algorithms use Extended or Unscented Kalman Filters to merge the sensor data, compensating for individual sensor inaccuracies and ensuring reliable position estimates even when GPS signals are lost.

### Dynamic Path Planning and Obstacle Avoidance

The rover follows a pre-planned global path but dynamically adjusts locally to avoid obstacles detected by sensors using reactive planning techniques. The path planner continuously evaluates terrain ruggedness, slope angles, and obstacle positions to select the safest and most efficient localized route.

### Blackout-Resilient Autonomous Navigation

When GPS or external control communication is lost, the rover automatically switches to blackout mode. In this mode:

- Global localization based on GPS is replaced with sensor fusion-based dead reckoning and visual odometry.
- Real-time terrain analysis allows local path replanning to navigate around new obstacles.
- The rover mimics human adaptive walking strategies to handle uncertain terrain, minimizing risks of tipping or tripping.

---

## Applications and Impact

This navigation system caters to multiple critical domains:

- **Mars or Lunar Exploration:** Enables self-sufficient rovers that can explore remote areas despite communication blackouts.
- **Military Robotics:** Operates autonomous unmanned ground vehicles in GPS-denied or signal-jammed battlefield zones.
- **Disaster Relief:** Assists in autonomous search and rescue in GPS-obstructed rubble fields or hazardous environments.
- **Precision Agriculture:** Supports autonomous agricultural vehicles working on uneven farmlands without reliance on satellite positioning.

---

## Technology Stack

| Component             | Description                                        | Tools/Frameworks      |
|-----------------------|--------------------------------------------------|----------------------|
| Programming Languages  | Core control and algorithm implementation         | Python, C++          |
| Robot Middleware      | Robot operating system for sensor and actuator integration | ROS, ROS2            |
| Sensor Fusion Methods | Algorithms for integrating multi-sensor data     | EKF, UKF Kalman Filters |
| Perception             | Visual and range sensing for terrain and obstacles | OpenCV, LIDAR drivers |
| Path Planning          | Algorithms for dynamic navigation                 | Custom planners, ROS navigation stack |
| Simulation & Testing   | Environment to test navigation in rough terrain  | Gazebo, RViz         |

---

## Getting Started

### Prerequisites

- ROS or ROS2 installed on your system
- Python 3.8+ and C++17 compatible compiler
- Sensor drivers for LIDAR, IMU, and camera modules

### Installation

1. Clone the repository:

   ```
   git clone https://github.com/yourusername/mars-rover-autonomous-navigation.git
   cd mars-rover-autonomous-navigation
   ```

2. Install dependencies:

   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```
   colcon build
   source install/setup.bash
   ```

### Usage

- Launch simulations in Gazebo using the provided world files.
- Deploy and test sensor fusion and path planning modules on hardware.
- Observe autonomous rover behavior during GPS blackouts in simulation or controlled field tests.

---

## Future Directions

- Integration of simultaneous localization and mapping (SLAM) for unknown terrain mapping during blackouts.
- Enhanced machine learning algorithms for terrain classification and navigation improvement.
- Development of cooperative multi-rover navigation for complex mission scenarios.
- Extended testing on physical rover platforms and planetary analog sites.

---

## Acknowledgments & Credits

This project was inspired by pioneering research in autonomous robotics, sensor fusion, and planetary exploration by NASA, Mars Society, and the open-source robotic community. Special thanks to the developers contributing to ROS, Gazebo, and Kalman filtering libraries for robotics.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

Made with passion for space exploration and robotic autonomy.

```


