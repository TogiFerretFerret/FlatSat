import time
import math
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL

class IMU:
    def __init__(self):
        self.ready = False
        try:
            self.i2c = board.I2C()
            self.accel_gyro = LSM6DS(self.i2c)
            self.mag = LIS3MDL(self.i2c)
            self.ready = True
            print("[Hardware] IMU (LSM6DSOX + LIS3MDL) Initialized.")
        except Exception as e:
            print(f"[Hardware] IMU Init Failed: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler Angles (in Radians) to Quaternion [w, x, y, z]
        """
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]

    def get_data(self):
        data = {
            "accel": {"x": 0, "y": 0, "z": 0},
            "gyro":  {"x": 0, "y": 0, "z": 0},
            "mag":   {"x": 0, "y": 0, "z": 0},
            "orientation_euler": {"roll": 0, "pitch": 0, "yaw": 0},
            "quaternion": [1, 0, 0, 0], # Identity Quaternion
            "temp": 0,
            "status": "OFFLINE"
        }

        if self.ready:
            try:
                # 1. Read Raw Sensors
                # accel is in m/s^2
                ax, ay, az = self.accel_gyro.acceleration 
                # gyro is in rad/s by default in CircuitPython, usually converted to deg/s
                gx, gy, gz = self.accel_gyro.gyro 
                # mag is in uT
                mx, my, mz = self.mag.magnetic
                
                # 2. Calculate Tilt (Basic Trig)
                roll_rad = math.atan2(ay, az)
                pitch_rad = math.atan2(-ax, math.sqrt(ay*ay + az*az))

                # 3. Calculate Heading (Tilt Compensated)
                mx_comp = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
                my_comp = mx * math.sin(roll_rad) * math.sin(pitch_rad) + \
                          my * math.cos(roll_rad) - \
                          mz * math.sin(roll_rad) * math.cos(pitch_rad)
                yaw_rad = math.atan2(-my_comp, mx_comp)

                # 4. Pack Data
                data["accel"] = {"x": ax, "y": ay, "z": az}
                # Convert Gyro to Degrees/sec for readability
                data["gyro"]  = {"x": math.degrees(gx), "y": math.degrees(gy), "z": math.degrees(gz)}
                data["mag"]   = {"x": mx, "y": my, "z": mz}
                data["temp"]  = self.accel_gyro.temperature
                
                data["orientation_euler"] = {
                    "roll": math.degrees(roll_rad),
                    "pitch": math.degrees(pitch_rad),
                    "yaw": math.degrees(yaw_rad)
                }
                
                # 5. Generate Quaternion (Vital for Physics Engine)
                data["quaternion"] = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                
                data["status"] = "ONLINE"

            except Exception as e:
                print(f"[IMU] Read Error: {e}")
                data["status"] = "ERROR"

        return data
