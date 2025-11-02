import time
import random

# Local project modules
from sensor_fusion import SensorFusionEKF
from blackout_nav import BlackoutNavigator

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

        print(f"[STEP {step}] Mode: {mode} | Pos: {traj_point} | Terrain: {camera['terrain']}")

        # Sleep to simulate real-time loop
        time.sleep(0.1)

    print('[MarsRover-AutoNav-Blackout] Navigation Complete. Final Trajectory Sample:', trajectory[-1])

if __name__ == '__main__':
    main()
