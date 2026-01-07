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
class PositionTracker:
    def __init__(self):
        self.active = False
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        # We track our OWN quaternion state to avoid accelerometer noise corrupting tilt
        self.q = [1.0, 0.0, 0.0, 0.0] # w, x, y, z
        self.last_time = time.time()

    def start(self, start_quat):
        # Capture initial orientation from the static sensor fusion
        # This gives us a good "Down" reference before we start moving
        self.q = start_quat if start_quat else [1.0, 0.0, 0.0, 0.0]
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.world_bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.time()
        self.active = True
        
        # We don't set bias immediately here; we let the adaptive loop handle it
        # This avoids capturing a "jerk" as the permanent bias
        print(f"[Physics] INS Started.")

    def stop(self):
        self.active = False
        print("[Physics] INS Stopped.")

    def update(self, curr_accel, curr_gyro, dt):
        if not self.active: return

        # --- 1. ORIENTATION UPDATE (Gyro Integration) ---
        # This decouples tilt from linear acceleration
        gx = math.radians(curr_gyro["x"])
        gy = math.radians(curr_gyro["y"])
        gz = math.radians(curr_gyro["z"])

        # Quaternion Derivative: q_dot = 0.5 * q * omega
        q = self.q
        q_dot = [
            0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz),
            0.5 * ( q[0]*gx + q[2]*gz - q[3]*gy),
            0.5 * ( q[0]*gy - q[1]*gz + q[3]*gx),
            0.5 * ( q[0]*gz + q[1]*gy - q[2]*gx)
        ]

        # Integrate
        self.q = [
            q[0] + q_dot[0] * dt,
            q[1] + q_dot[1] * dt,
            q[2] + q_dot[2] * dt,
            q[3] + q_dot[3] * dt
        ]

        # Normalize Quaternion (Critical to prevent drift explosion)
        norm = math.sqrt(sum(x*x for x in self.q))
        if norm > 0:
            self.q = [x / norm for x in self.q]

        # --- 2. ROTATE TO WORLD FRAME ---
        ax_s, ay_s, az_s = curr_accel['x'], curr_accel['y'], curr_accel['z']
        qw, qx, qy, qz = self.q
        
        # Optimized rotation logic
        xx = qx * qx; yy = qy * qy; zz = qz * qz
        xy = qx * qy; xz = qx * qz; yz = qy * qz
        wx = qw * qx; wy = qw * qy; wz = qw * qz

        ax_world = ax_s * (1.0 - 2.0 * (yy + zz)) + ay_s * (2.0 * (xy - wz)) + az_s * (2.0 * (xz + wy))
        ay_world = ax_s * (2.0 * (xy + wz)) + ay_s * (1.0 - 2.0 * (xx + zz)) + az_s * (2.0 * (yz - wx))
        az_world = ax_s * (2.0 * (xz - wy)) + ay_s * (2.0 * (yz + wx)) + az_s * (1.0 - 2.0 * (xx + yy))

        # --- 3. ADAPTIVE BIAS & ZUPT (The Fix) ---
        
        # Calculate Magnitude. Should be ~9.81 if still.
        current_mag = math.sqrt(ax_world**2 + ay_world**2 + az_world**2)
        
        # If we are close to 1G (within 0.5 m/s^2), assume stationary.
        is_stationary = abs(current_mag - 9.81) < 0.5

        if is_stationary:
            # ADAPTIVE BIAS: Slowly shift the zero-point to match current reading.
            # This kills the "climbing negative Z" by accepting the current Z as the new zero.
            alpha = 0.05 # Adaptation speed (0.0 to 1.0)
            
            # Target bias is the current reading (minus true gravity for Z)
            self.world_bias["x"] = self.world_bias["x"] * (1-alpha) + (ax_world) * alpha
            self.world_bias["y"] = self.world_bias["y"] * (1-alpha) + (ay_world) * alpha
            self.world_bias["z"] = self.world_bias["z"] * (1-alpha) + (az_world - 9.81) * alpha
            
            # Kill velocity aggressively when stationary
            self.vel["x"] *= 0.5
            self.vel["y"] *= 0.5
            self.vel["z"] *= 0.5
            
            # Zero out if very small
            if abs(self.vel["x"]) < 0.01: self.vel["x"] = 0
            if abs(self.vel["y"]) < 0.01: self.vel["y"] = 0
            if abs(self.vel["z"]) < 0.01: self.vel["z"] = 0
            
            # Don't accelerate
            ax, ay, az = 0, 0, 0
        else:
            # We are moving. Subtract the learned bias.
            ax = ax_world - self.world_bias["x"]
            ay = ay_world - self.world_bias["y"]
            az = (az_world - 9.81) - self.world_bias["z"]

            # Deadband Filter
            threshold = 0.2
            if abs(ax) < threshold: ax = 0
            if abs(ay) < threshold: ay = 0
            if abs(az) < threshold: az = 0

            # Integrate Acceleration -> Velocity
            self.vel["x"] += ax * dt
            self.vel["y"] += ay * dt
            self.vel["z"] += az * dt
            
            # Friction (Drag)
            self.vel["x"] *= 0.99
            self.vel["y"] *= 0.99
            self.vel["z"] *= 0.90 # Heavy Z drag to stop vertical jitter

        # Integrate Velocity -> Position
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
        print(f"[System] Camera Active at {width}x{height}.")
    except Exception as e:
        print(f"[System] Camera Config Failed: {e}")

# FORCE 1080p STARTUP
configure_camera(1920, 1080)

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

                last_loop_time = time.time()

                while self.running:
                    # Calculate accurate DT for integration
                    now = time.time()
                    dt = now - last_loop_time
                    last_loop_time = now

                    data = imu.get_data() if imu else {"status": "no_imu"}
                    
                    if data["status"] == "ONLINE":
                        # PASS GYRO DATA TO TRACKER
                        tracker.update(data["accel"], data["gyro"], dt)
                        
                        data["pos_offset"] = tracker.pos
                        data["tracking_active"] = tracker.active

                        # Calculate Derived Magnitudes for UI
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
                    
                    # Target ~100Hz loop for smooth physics
                    # We sleep less to ensure we catch fast movements
                    time.sleep(0.005) 
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
                        # Initialize tracker with current IMU orientation
                        tracker.start(data["quaternion"])
                elif cmd == "TRK-":
                    tracker.stop()
            except: break

# --- FLASK ROUTES ---
@app.route('/stream')
def stream():
    """ 1080p Stream for Aiming """
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
    """ 4.6K Snapshot then reverts to 1080p """
    stream = io.BytesIO()
    with cam_lock:
        print("[Snap] Switching to 4.6K...")
        configure_camera(4608, 2592)
        cam.picam2.capture_file(stream, format="jpeg")
        
        print("[Snap] Reverting to 1080p...")
        configure_camera(1920, 1080)
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
