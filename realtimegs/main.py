import socket
import threading
import time
import json
import struct
import os
import io
import sys
import subprocess
import math
from flask import Flask, Response, send_file

# --- HARDWARE IMPORTS ---
try:
    from hardware.camera import CameraSystem
    from hardware.imu import IMU
except ImportError:
    print("Hardware libraries missing. Ensure hardware/ folder exists.")
    sys.exit(1)

# --- CONFIG ---
HTTP_PORT = 8000 

# Initialize Global Hardware
cam = CameraSystem()
imu = IMU()
cam_lock = threading.Lock()

# --- ADVANCED PHYSICS ENGINE (INS) ---
def rotate_vector_by_quaternion(v, q):
    """ Rotates vector v (x,y,z) by quaternion q (w,x,y,z) into World Frame. """
    vx, vy, vz = v['x'], v['y'], v['z']
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    x1 = qy * vz - qz * vy
    y1 = qz * vx - qx * vz
    z1 = qx * vy - qy * vx

    x2 = qy * z1 - qz * y1
    y2 = qz * x1 - qx * z1
    z2 = qx * y1 - qy * x1

    return {
        "x": vx + 2.0 * (qw * x1 + x2),
        "y": vy + 2.0 * (qw * y1 + y2),
        "z": vz + 2.0 * (qw * z1 + z2)
    }

class PositionTracker:
    def __init__(self):
        self.active = False
        self.calibrating = False
        self.calibration_samples = 0
        self.calibration_sum = {"x":0, "y":0, "z":0}
        
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.q = [1.0, 0.0, 0.0, 0.0] 
        self.world_bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.time()

    def start_calibration(self):
        """ Start collecting samples for zero-point training """
        self.calibrating = True
        self.calibration_samples = 0
        self.calibration_sum = {"x":0, "y":0, "z":0}
        print("[Physics] CALIBRATION STARTED (Collecting samples)...")

    def stop_calibration(self):
        """ Stop collecting and compute the average bias """
        if not self.calibrating: return
        
        if self.calibration_samples > 0:
            self.world_bias = {
                "x": self.calibration_sum["x"] / self.calibration_samples,
                "y": self.calibration_sum["y"] / self.calibration_samples,
                "z": self.calibration_sum["z"] / self.calibration_samples
            }
            print(f"[Physics] CALIBRATION COMPLETE. Samples: {self.calibration_samples}. New Zero: {self.world_bias}")
        else:
            print("[Physics] CALIBRATION STOPPED (No samples). Zero unchanged.")
            
        self.calibrating = False

    def start(self, start_quat):
        # Start tracking relative to the PREVIOUSLY trained bias
        self.q = start_quat if start_quat else [1.0, 0.0, 0.0, 0.0]
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.time()
        self.active = True
        print(f"[Physics] TRACKING STARTED. Using Bias: {self.world_bias}")

    def stop(self):
        self.active = False
        print("[Physics] STOPPED.")

    def update(self, curr_accel, curr_gyro, dt):
        # --- 1. ORIENTATION UPDATE ---
        gx = math.radians(curr_gyro["x"])
        gy = math.radians(curr_gyro["y"])
        gz = math.radians(curr_gyro["z"])

        q = self.q
        q_dot = [
            0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz),
            0.5 * ( q[0]*gx + q[2]*gz - q[3]*gy),
            0.5 * ( q[0]*gy - q[1]*gz + q[3]*gx),
            0.5 * ( q[0]*gz + q[1]*gy - q[2]*gx)
        ]

        self.q = [q[0] + q_dot[0] * dt, q[1] + q_dot[1] * dt, q[2] + q_dot[2] * dt, q[3] + q_dot[3] * dt]
        norm = math.sqrt(sum(x*x for x in self.q))
        if norm > 0: self.q = [x / norm for x in self.q]

        # --- 2. ROTATE TO WORLD FRAME ---
        world_raw = rotate_vector_by_quaternion(curr_accel, self.q)

        # --- 3. CALIBRATION MODE (The "Train" Phase) ---
        if self.calibrating:
            self.calibration_sum["x"] += world_raw["x"]
            self.calibration_sum["y"] += world_raw["y"]
            self.calibration_sum["z"] += world_raw["z"]
            self.calibration_samples += 1
            return 

        if not self.active: return

        # --- 4. PHYSICS INTEGRATION ---
        # Calculate dynamic acceleration (movement) by removing the trained bias (gravity/offset)
        ax = world_raw["x"] - self.world_bias["x"]
        ay = world_raw["y"] - self.world_bias["y"]
        az = world_raw["z"] - self.world_bias["z"]

        # --- CRITICAL FIX: MAGNITUDE-BASED ZUPT ---
        # Calculate total movement intensity
        dynamic_mag = math.sqrt(ax**2 + ay**2 + az**2)
        
        # Threshold: 0.35 m/s^2. 
        # If total acceleration is less than this, we assume we are stationary.
        # This prevents "gravity leak" (0.1 - 0.2 m/s^2) from integrating into infinity.
        THRESHOLD = 0.35 
        
        # Friction: 0.95.
        # Constant drag prevents velocity from exploding if a bias persists.
        FRICTION = 0.95 

        if dynamic_mag < THRESHOLD:
            # We are likely still. Kill velocity immediately.
            self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
            ax, ay, az = 0, 0, 0
        else:
            # We are moving. Integrate.
            pass

        # Integrate Accel -> Velocity
        self.vel["x"] += ax * dt
        self.vel["y"] += ay * dt
        self.vel["z"] += az * dt
        
        # Apply Drag
        self.vel["x"] *= FRICTION
        self.vel["y"] *= FRICTION
        self.vel["z"] *= FRICTION

        # Integrate Velocity -> Position
        self.pos["x"] += self.vel["x"] * dt
        self.pos["y"] += self.vel["y"] * dt
        self.pos["z"] += self.vel["z"] * dt

tracker = PositionTracker()

# --- CAMERA CONFIG ---
def configure_camera(width, height):
    try:
        try: cam.picam2.stop()
        except: pass
        config = cam.picam2.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
        cam.picam2.configure(config)
        cam.picam2.start()
    except Exception as e: print(f"[Cam] Config Error: {e}")

configure_camera(1920, 1080)

app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        try:
            cmd = ["hcitool", "rssi", mac]
            return int(subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode().split(":")[1])
        except: return 0

    def get_system_stats(self):
        stats = {"cpu_temp": 0, "cpu_load": 0}
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f: stats["cpu_temp"] = float(f.read()) / 1000.0
            stats["cpu_load"] = os.getloadavg()[0] * 25
        except: pass
        return stats

    def run_bt_server(self):
        print("[BT] Init...")
        os.system("sudo hciconfig hci0 piscan")
        os.system("sdptool add SP")
        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server_sock.bind(("00:00:00:00:00:00", 1))
        server_sock.listen(1)

        while self.running:
            try:
                client, addr = server_sock.accept()
                print(f"[BT] Connected: {addr}")
                threading.Thread(target=self.listen_commands, args=(client,), daemon=True).start()
                last_time = time.time()

                while self.running:
                    now = time.time()
                    dt = now - last_time
                    last_time = now
                    
                    data = imu.get_data() if imu else {"status": "no_imu"}
                    if data["status"] == "ONLINE":
                        tracker.update(data["accel"], data["gyro"], dt)
                        data["pos_offset"] = tracker.pos
                        data["calibrating"] = tracker.calibrating
                        data["tracking_active"] = tracker.active

                    data.update(self.get_system_stats())
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    msg = json.dumps(data).encode('utf-8')
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    try: client.sendall(header + msg)
                    except: break
                    time.sleep(0.005)
            except: time.sleep(1)

    def listen_commands(self, sock):
        while True:
            try:
                header = sock.recv(8)
                if not header: break
                _, length = struct.unpack("!4sI", header)
                cmd = struct.unpack("!4sI", header)[0].decode().strip()
                payload = sock.recv(length) 
                
                if cmd == "TRK+": 
                    data = imu.get_data()
                    tracker.start(data.get("quaternion"))
                elif cmd == "TRK-": tracker.stop()
                elif cmd == "CAL+": tracker.start_calibration()
                elif cmd == "CAL-": tracker.stop_calibration()
                
            except: break

@app.route('/stream')
def stream():
    def generate():
        stream = io.BytesIO()
        while True:
            with cam_lock: cam.picam2.capture_file(stream, format="jpeg")
            stream.seek(0)
            frame = stream.read()
            stream.seek(0)
            stream.truncate()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    stream = io.BytesIO()
    with cam_lock:
        configure_camera(4608, 2592)
        cam.picam2.capture_file(stream, format="jpeg")
        configure_camera(1920, 1080)
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
