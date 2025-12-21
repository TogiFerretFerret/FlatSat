import time
import sys
from config import HTTP_PORT, SHAKE_THRESHOLD
from hardware.camera import CameraSystem
from hardware.imu import IMU
from web import server

def main():
    print("--- ARTEMIS SWARM STARTUP ---")
    
    # 1. Hardware Init
    try:
        imu = IMU()
        cam = CameraSystem()
    except Exception as e:
        print(f"HW ERROR: {e}")
        return

    # 2. Start Web Server (Daemon Thread)
    print(f"Starting Ground Station on port {HTTP_PORT}...")
    server.run_server(cam, imu, HTTP_PORT)
    
    # 3. Main Loop
    print("Ready.")
    try:
        while True:
            # Get Data
            data = imu.get_data()
            if data['status'] == 'ONLINE':
                ax = data['accel']['x']
                ay = data['accel']['y']
                az = data['accel']['z']
                
                # Check for shake (>15m/s^2)
                if max(abs(ax), abs(ay), abs(az)) > SHAKE_THRESHOLD:
                    print(">>> SHAKE DETECTED")
                    
                    # Trigger capture via Web Server context
                    with server.app.app_context():
                        server.capture()
                    
                    # Cooldown
                    time.sleep(2)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        cam.stop()

if __name__ == "__main__":
    main()
