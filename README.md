# MarsRover-AutoNav-Blackout
| Component             | Description                                        | Tools/Frameworks      |
|-----------------------|--------------------------------------------------|-----------------------|
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
## Backend Database Logging
### RoverDB SQLite Interface
The rover uses a lightweight SQLite database (`RoverDB`) to persist mission telemetry and sensor data for post-mission analysis and debugging. This enables comprehensive data logging without requiring external database servers.

### Logged Data Tables
- **nav_log**: Records navigation events including waypoints, path decisions, obstacle detections, and autonomous maneuvers during GPS blackouts
- **sensor_data**: Stores timestamped sensor readings from LIDAR, IMU, cameras, and odometry data for sensor fusion validation

### Usage in main.py
To integrate database logging in your rover control script:
```python
from rover_db import RoverDB

# Initialize database connection
db = RoverDB('rover_mission.db')

# Log navigation events
db.log_navigation(
    timestamp=time.time(),
    waypoint_id=current_waypoint,
    action='obstacle_avoidance',
    gps_available=False,
    position=(x, y, z)
)

# Log sensor readings
db.log_sensor_data(
    timestamp=time.time(),
    sensor_type='lidar',
    data=lidar_reading,
    confidence=0.95
)

# Close connection when done
db.close()
```

### Database File Location
The SQLite database file is stored in the project root directory as `rover_mission.db` by default. You can specify a custom path when initializing the RoverDB instance. The database is automatically created if it doesn't exist.

### Analyzing Mission Telemetry
Use standard SQLite tools or Python to query logged data:
```sql
-- Retrieve all navigation events during GPS blackout
SELECT timestamp, waypoint_id, action, position 
FROM nav_log 
WHERE gps_available = 0 
ORDER BY timestamp DESC;

-- Analyze LIDAR sensor performance
SELECT AVG(confidence) as avg_confidence, COUNT(*) as reading_count
FROM sensor_data 
WHERE sensor_type = 'lidar'
GROUP BY DATE(timestamp);
```

For Python-based analysis:
```python
import sqlite3
import pandas as pd

conn = sqlite3.connect('rover_mission.db')
df = pd.read_sql_query("SELECT * FROM nav_log", conn)
print(df.describe())
conn.close()
```
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
