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

# --- PHYSICS ENGINE ---
def rotate_vector_by_quaternion(v, q):
    """
    Rotates vector v (x,y,z) by quaternion q (w,x,y,z).
    Returns rotated vector [x, y, z].
    """
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
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.world_bias = {"x": 0.0, "y": 0.0, "z": 0.0} 
        self.last_time = time.time()

    def start(self, current_accel, current_quat):
        world_a = rotate_vector_by_quaternion(current_accel, current_quat)
        
        # Tare: Capture gravity + bias at the start
        self.world_bias = {
            "x": world_a["x"] * 9.81,
            "y": world_a["y"] * 9.81,
            "z": world_a["z"] * 9.81
        }
        
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.time()
        self.active = True
        print(f"[Physics] Tracking Started. World Bias: {self.world_bias}")

    def stop(self):
        self.active = False
        print("[Physics] Tracking Stopped.")

    def update(self, curr_accel, curr_quat):
        if not self.active: return
        
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # 1. Rotate to World Frame
        world_raw = rotate_vector_by_quaternion(curr_accel, curr_quat)
        
        # 2. Convert to m/s^2 & Remove Bias
        ax = (world_raw["x"] * 9.81) - self.world_bias["x"]
        ay = (world_raw["y"] * 9.81) - self.world_bias["y"]
        az = (world_raw["z"] * 9.81) - self.world_bias["z"]

        # 3. Smart Deadband & Friction (THE FIX)
        # If acceleration is tiny (noise), kill velocity so position stops drift.
        threshold = 0.25 # Slightly higher threshold for stability
        
        if abs(ax) < threshold: 
            ax = 0
            self.vel["x"] *= 0.85 # Strong friction when stopped
        else:
            self.vel["x"] *= 0.99 # Tiny air resistance when moving

        if abs(ay) < threshold: 
            ay = 0
            self.vel["y"] *= 0.85
        else:
            self.vel["y"] *= 0.99

        if abs(az) < threshold: 
            az = 0
            self.vel["z"] *= 0.85 
        else:
            self.vel["z"] *= 0.99

        # 4. Integrate: Accel -> Velocity
        self.vel["x"] += ax * dt
        self.vel["y"] += ay * dt
        self.vel["z"] += az * dt
        
        # 5. Integrate: Velocity -> Position
        self.pos["x"] += self.vel["x"] * dt
        self.pos["y"] += self.vel["y"] * dt
        self.pos["z"] += self.vel["z"] * dt

tracker = PositionTracker()

# --- CAMERA CONFIG ---
def configure_camera(width, height):
    print(f"[System] Switching Camera to {width}x{height}...")
    try:
        try: cam.picam2.stop()
        except: pass
        config = cam.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        cam.picam2.configure(config)
        cam.picam2.start()
    except Exception as e:
        print(f"[System] Camera Config Failed: {e}")

configure_camera(1920, 1080) # Startup at 1080p

app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        
        # State Cache
        self.cached_stats = {
            "cpu_temp": 0, "cpu_load": 0, "mem_percent": 0, 
            "disk_percent": 0, "uptime": 0, "wifi_signal": 0, 
            "throttled": "0x0", "voltage": 0, "clock_mhz": 0
        }
        self.last_stats_time = 0
        self.cached_rssi = 0
        self.last_rssi_time = 0

        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        if time.time() - self.last_rssi_time < 2.0: return self.cached_rssi
        try:
            cmd = ["hcitool", "rssi", mac]
            res = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            val = int(res.decode().split(":")[1])
            self.cached_rssi = val
            self.last_rssi_time = time.time()
            return val
        except: return self.cached_rssi

    def get_system_stats(self):
        if time.time() - self.last_stats_time < 2.0: return self.cached_stats
        stats = self.cached_stats.copy()
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                stats["cpu_temp"] = float(f.read()) / 1000.0
            
            cpu_count = os.cpu_count() or 1
            stats["cpu_load"] = round(os.getloadavg()[0] * 100 / cpu_count, 1)
            
            with open("/proc/meminfo", "r") as f:
                lines = f.readlines()
                total = int(lines[0].split()[1])
                available = int(lines[2].split()[1])
                stats["mem_percent"] = round((1 - available/total) * 100, 1)

            st = os.statvfs('/')
            total = st.f_blocks * st.f_frsize
            free = st.f_bavail * st.f_frsize
            stats["disk_percent"] = round((1 - free/total) * 100, 1)
            
            with open("/proc/uptime", "r") as f:
                stats["uptime"] = int(float(f.read().split()[0]))

            with open("/proc/net/wireless", "r") as f:
                for line in f:
                    if "wlan" in line:
                        stats["wifi_signal"] = int(float(line.split()[3]))
                        break

            res = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL)
            stats["throttled"] = res.decode().strip().split('=')[1]

            res = subprocess.check_output(["vcgencmd", "measure_volts", "core"], stderr=subprocess.DEVNULL)
            stats["voltage"] = float(res.decode().strip().split('=')[1].replace('V', ''))

            res = subprocess.check_output(["vcgencmd", "measure_clock", "arm"], stderr=subprocess.DEVNULL)
            stats["clock_mhz"] = int(res.decode().strip().split('=')[1]) // 1000000
            
        except Exception: pass
        
        self.cached_stats = stats
        self.last_stats_time = time.time()
        return stats

    def run_bt_server(self):
        print("[BT] Configuring Adapter...")
        os.system("sudo hciconfig hci0 piscan")
        os.system("sdptool add SP")

        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server_sock.bind(("00:00:00:00:00:00", 1))
        server_sock.listen(1)
        print("[BT] Ready...")

        while self.running:
            try:
                client, addr = server_sock.accept()
                print(f"[BT] Connected: {addr}")
                
                cmd_thread = threading.Thread(target=self.listen_commands, args=(client,))
                cmd_thread.daemon = True
                cmd_thread.start()

                while self.running:
                    data = imu.get_data() if imu else {"status": "no_imu"}
                    
                    if data["status"] == "ONLINE":
                        tracker.update(data["accel"], data["quaternion"])
                        data["pos_offset"] = tracker.pos
                        data["tracking_active"] = tracker.active

                        # Calculate Derived Magnitudes
                        ax, ay, az = data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
                        data["accel_mag"] = math.sqrt(ax**2 + ay**2 + az**2)
                        
                        gx, gy, gz = data["gyro"]["x"], data["gyro"]["y"], data["gyro"]["z"]
                        data["gyro_mag"] = math.sqrt(gx**2 + gy**2 + gz**2)
                        
                        mx, my, mz = data["mag"]["x"], data["mag"]["y"], data["mag"]["z"]
                        data["mag_mag"] = math.sqrt(mx**2 + my**2 + mz**2)

                    sys_stats = self.get_system_stats()
                    data.update(sys_stats)
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    msg = json.dumps(data).encode('utf-8')
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    try: client.sendall(header + msg)
                    except BrokenPipeError: break
                    time.sleep(0.05)
            except Exception as e:
                print(f"[BT] Connection Reset: {e}")
                time.sleep(1)

    def listen_commands(self, sock):
        while True:
            try:
                header = sock.recv(8)
                if not header: break
                type_bytes, length = struct.unpack("!4sI", header)
                payload = b''
                while len(payload) < length:
                    chunk = sock.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                cmd = type_bytes.decode('utf-8').strip()
                if cmd == "TRK+":
                    data = imu.get_data()
                    if data["status"] == "ONLINE":
                        tracker.start(data["accel"], data["quaternion"])
                elif cmd == "TRK-":
                    tracker.stop()
            except: break

# --- FLASK ROUTES ---
@app.route('/stream')
def stream():
    def generate():
        stream = io.BytesIO()
        while True:
            with cam_lock:
                cam.picam2.capture_file(stream, format="jpeg")
            stream.seek(0)
            frame = stream.read()
            stream.seek(0)
            stream.truncate()
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    stream = io.BytesIO()
    with cam_lock:
        print("[Snap] 4.6K...")
        configure_camera(4608, 2592)
        cam.picam2.capture_file(stream, format="jpeg")
        print("[Snap] Reverting...")
        configure_camera(1920, 1080)
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
