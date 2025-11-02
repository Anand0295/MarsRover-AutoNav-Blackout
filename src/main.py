import time
import random

# Local project modules
from sensor_fusion import SensorFusionEKF
from blackout_nav import BlackoutNavigator
from backend.db_interface import RoverDB


# Simulated perception stubs
def get_imu_data():
    # Return (ax, ay, az, wx, wy, wz)
    return [random.gauss(0, 0.1) for _ in range(6)]


def get_lidar_data():
    # Return list of detected obstacle distances/angles
    return [random.uniform(0.2, 5.0) for _ in range(8)]


def get_camera_data():
    # Simulate detection (could be terrain class, etc)
    return {'terrain': random.choice(['rock', 'sand', 'slope', 'flat'])}


def gps_available():
    # Simulate GPS/comm blackout toggling
    return random.choice([True] * 7 + [False] * 3)


def main():
    print('[MarsRover-AutoNav-Blackout] Autonomous Navigation Demo Started')
    
    # Initialize database for persistent logging
    db = RoverDB("rover_mission.db")
    print(f'Database initialized: {db.db_path}')
    
    # Initial state vector: x, y, heading, vx, vy (example values)
    initial_state = [0, 0, 0, 0, 0]
    initial_cov = [[1,0,0,0,0],[0,1,0,0,0],[0,0,0.5,0,0],[0,0,0,0.2,0],[0,0,0,0,0.2]]
    
    fusion = SensorFusionEKF(initial_state, initial_cov)
    nav = BlackoutNavigator(fusion)
    
    trajectory = []
    
    for step in range(100):
        imu = get_imu_data()
        lidar = get_lidar_data()
        camera = get_camera_data()
        gps_ok = gps_available()
        
        # Build sensor observation vector (merge data as needed in practice)
        observation = imu + lidar  # Would be preprocessed for EKF
        
        # Navigation logic
        if gps_ok:
            # Simulate global navigation with GPS
            # F and Q, H and R, and z would come from the system model and sensors
            F = Q = H = R = None  # Replace with real matrices
            z = observation  # Replace with real sensor data
            # fusion.predict(F, Q)
            # fusion.update(z, H, R)
            mode = 'GPS/NORMAL'
        else:
            # Blackout fallback—dead reckoning using IMU/odometry
            nav_state = nav.run(sensor_inputs=imu)
            mode = 'BLACKOUT'
        
        # Store trajectory; for demo, just use fusion or nav_state
        traj_point = fusion.get_state()[0] if gps_ok else nav_state
        trajectory.append(traj_point)
        
        # Log navigation data to database
        state = fusion.get_state()
        position = (state[0], state[1], 0.0)  # x, y, z (z=0 for 2D)
        velocity = (state[3], state[4], 0.0)  # vx, vy, vz (vz=0 for 2D)
        heading = state[2]  # heading angle
        
        db.log_navigation(position, velocity, heading, status=mode)
        
        # Log sensor data to database
        temperature = random.uniform(-60, 20)  # Simulated Mars temperature
        battery_level = random.uniform(60, 100)  # Battery percentage
        obstacle_detected = min(lidar) < 1.0  # Check if obstacle is close
        distance_to_obstacle = min(lidar) if obstacle_detected else None
        
        sensor_metadata = {
            'terrain': camera['terrain'],
            'imu_samples': len(imu),
            'lidar_samples': len(lidar)
        }
        
        db.log_sensor_data(
            temperature, 
            battery_level, 
            obstacle_detected, 
            distance_to_obstacle,
            metadata=sensor_metadata
        )
        
        # Periodic status update
        if step % 10 == 0:
            print(f'Step {step}: Mode={mode}, Position=({position[0]:.2f}, {position[1]:.2f}), '
                  f'Battery={battery_level:.1f}%, Obstacles={obstacle_detected}')
        
        time.sleep(0.05)  # Simulate real-time processing
    
    # Display database statistics
    stats = db.get_stats()
    print('\n=== Mission Summary ===')
    print(f'Navigation records logged: {stats["navigation_records"]}')
    print(f'Sensor records logged: {stats["sensor_records"]}')
    print(f'Database saved to: {stats["database_path"]}')
    
    # Show recent navigation data
    print('\n=== Recent Navigation Data ===')
    recent_nav = db.get_recent_navigation(limit=5)
    for record in recent_nav:
        print(f'  {record["timestamp"]}: Position=({record["position_x"]:.2f}, {record["position_y"]:.2f}), '
              f'Status={record["status"]}')
    
    db.close()
    print('\n[MarsRover-AutoNav-Blackout] Demo Complete. Trajectory saved to database.')


if __name__ == '__main__':
    main()
