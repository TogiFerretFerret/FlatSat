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

# --- SYSTEM MONITOR (The Kitchen Sink) ---
class SystemMonitor:
    def __init__(self):
        self.cached_stats = {}
        self.last_slow_update = 0
        self.start_time = time.time()

    def get_quick_stats(self):
        """Stats that change fast and are cheap to read"""
        stats = {}
        # CPU Temp
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                stats["cpu_temp"] = round(float(f.read()) / 1000.0, 1)
        except: stats["cpu_temp"] = 0

        # CPU Load (1 min avg)
        try:
            stats["cpu_load"] = round(os.getloadavg()[0] / os.cpu_count() * 100, 1)
        except: stats["cpu_load"] = 0
        
        return stats

    def get_slow_stats(self):
        """Heavy stats, cached for 2 seconds"""
        if time.time() - self.last_slow_update < 2.0:
            return self.cached_stats
        
        stats = {}
        
        # 1. Voltage & Clock & Throttle (vcgencmd)
        try:
            # Core Voltage
            res = subprocess.check_output(["vcgencmd", "measure_volts", "core"], stderr=subprocess.DEVNULL)
            stats["cpu_volts"] = float(res.decode().split("=")[1].replace("V\n", ""))
            
            # Clock Speed
            res = subprocess.check_output(["vcgencmd", "measure_clock", "arm"], stderr=subprocess.DEVNULL)
            stats["cpu_clock"] = int(int(res.decode().split("=")[1]) / 1000000)
            
            # Throttling Hex
            res = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL)
            stats["throttle_hex"] = res.decode().split("=")[1].strip()
        except: 
            stats["cpu_volts"] = 0
            stats["cpu_clock"] = 0
            stats["throttle_hex"] = "0x0"

        # 2. Memory Usage
        try:
            with open("/proc/meminfo", "r") as f:
                mem = {}
                for line in f:
                    parts = line.split()
                    mem[parts[0].replace(":", "")] = int(parts[1])
                
                total = mem.get("MemTotal", 1)
                avail = mem.get("MemAvailable", 0)
                stats["ram_total_mb"] = int(total / 1024)
                stats["ram_used_mb"] = int((total - avail) / 1024)
                stats["ram_percent"] = round(((total - avail) / total) * 100, 1)
        except: pass

        # 3. Disk Usage
        try:
            st = os.statvfs('/')
            total = st.f_blocks * st.f_frsize
            free = st.f_bavail * st.f_frsize
            stats["disk_total_gb"] = round(total / (1024**3), 1)
            stats["disk_used_gb"] = round((total - free) / (1024**3), 1)
            stats["disk_percent"] = round(((total - free) / total) * 100, 1)
        except: pass

        # 4. Uptime
        try:
            with open("/proc/uptime", "r") as f:
                stats["uptime_sys"] = int(float(f.read().split()[0]))
        except: stats["uptime_sys"] = 0
        
        stats["uptime_script"] = int(time.time() - self.start_time)

        # 5. Wireless
        try:
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
                if len(lines) > 2:
                    # wlan0: 0000   50.  -60.  ...
                    parts = lines[2].split()
                    stats["wifi_dbm"] = int(float(parts[3]))
                    stats["wifi_link"] = int(float(parts[2]))
        except: 
            stats["wifi_dbm"] = 0
            stats["wifi_link"] = 0

        # 6. Top Processes (New)
        try:
            # Fetches PID, Command, %CPU, %MEM sorted by CPU usage
            cmd = ["ps", "-Ao", "pid,comm,pcpu,pmem", "--sort=-pcpu"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode()
            lines = output.strip().split('\n')
            
            procs = []
            # Skip header, take top 5
            for line in lines[1:6]:
                parts = line.split()
                if len(parts) >= 4:
                    procs.append({
                        "pid": parts[0],
                        "name": parts[1],
                        "cpu": parts[2],
                        "mem": parts[3]
                    })
            stats["processes"] = procs
        except:
            stats["processes"] = []

        self.cached_stats = stats
        self.last_slow_update = time.time()
        return stats

sys_mon = SystemMonitor()

# --- ATTITUDE MATH ---
def calculate_attitude(accel, mag):
    """ Calculate Roll, Pitch, and Yaw from Raw Sensors """
    ax, ay, az = accel['x'], accel['y'], accel['z']
    
    # Roll (Rotation around X)
    roll = math.atan2(ay, az)
    
    # Pitch (Rotation around Y)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    
    # Yaw (Compass Heading)
    mx, my, mz = mag['x'], mag['y'], mag['z']
    
    # Tilt compensation
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    
    mx_comp = mx * cos_p + mz * sin_p
    my_comp = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p
    
    yaw = math.atan2(-my_comp, mx_comp)
    
    # Convert to degrees
    return {
        "roll": math.degrees(roll),
        "pitch": math.degrees(pitch),
        "yaw": (math.degrees(yaw) + 360) % 360
    }

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
        
        self.cached_rssi = 0
        self.last_rssi_time = 0
        
        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        if time.time() - self.last_rssi_time < 2.0: return self.cached_rssi
        try:
            cmd = ["hcitool", "rssi", mac]
            val = int(subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode().split(":")[1])
            self.cached_rssi = val
            self.last_rssi_time = time.time()
            return val
        except: return self.cached_rssi

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

                while self.running:
                    # 1. Gather Sensor Data
                    data = imu.get_data() if imu else {"status": "no_imu"}
                    
                    if data["status"] == "ONLINE":
                        # Calculate high-res attitude
                        att = calculate_attitude(data["accel"], data["mag"])
                        data["attitude"] = att
                        
                        # Total G calculation
                        ax, ay, az = data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
                        data["total_g"] = round(math.sqrt(ax**2 + ay**2 + az**2) / 9.81, 2)

                    # 2. Gather System Data
                    # Merge Quick stats (every frame) and Slow stats (cached)
                    sys_data = sys_mon.get_quick_stats()
                    sys_data.update(sys_mon.get_slow_stats())
                    data["sys"] = sys_data
                    
                    # 3. Connectivity
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    # 4. Transmit
                    msg = json.dumps(data).encode('utf-8')
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    try: client.sendall(header + msg)
                    except: break
                    
                    time.sleep(0.05) # 20Hz update
            except: time.sleep(1)

    def listen_commands(self, sock):
        while True:
            try:
                header = sock.recv(8)
                if not header: break
                _, length = struct.unpack("!4sI", header)
                payload = sock.recv(length)
            except: break

# --- FLASK ROUTES ---
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
