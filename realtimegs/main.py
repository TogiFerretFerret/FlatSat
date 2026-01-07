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

# --- SYSTEM MONITOR ---
class SystemMonitor:
    def __init__(self):
        # FIX: Initialize with FULL DEFAULTS so the first calls don't return {}
        self.cached_stats = {
            "cpu_volts": 0, "cpu_clock": 0, "throttle_hex": "0x0", "throttle_flags": ["INIT"],
            "gpu_mem": "0M", 
            "ram_total_mb": 0, "ram_used_mb": 0, "ram_percent": 0,
            "disk_total_gb": 0, "disk_used_gb": 0, "disk_percent": 0,
            "uptime_sys": 0, "uptime_script": 0, "ip_addr": "Unknown",
            "wifi_dbm": 0, "wifi_link": 0, "wifi_retry": 0, "wifi_missed": 0,
            "processes": []
        }
        self.last_slow_update = 0
        self.start_time = time.time()
        
        # Network Speed Tracking
        self.last_net_time = time.time()
        self.last_rx = 0
        self.last_tx = 0
        self.net_stats = {"rx_kbps": 0, "tx_kbps": 0}

    def decode_throttle(self, hex_str):
        try:
            code = int(hex_str, 16)
            flags = []
            if code & 0x1: flags.append("UNDERVOLT")
            if code & 0x2: flags.append("FREQ_CAP")
            if code & 0x4: flags.append("THROTTLED")
            if code & 0x8: flags.append("SOFT_TEMP")
            return flags if flags else ["NOMINAL"]
        except: return ["UNKNOWN"]

    def update_network_stats(self):
        try:
            iface = "wlan0"
            if not os.path.exists(f"/sys/class/net/{iface}"):
                iface = "eth0"
            
            with open(f"/sys/class/net/{iface}/statistics/rx_bytes", "r") as f: rx = int(f.read())
            with open(f"/sys/class/net/{iface}/statistics/tx_bytes", "r") as f: tx = int(f.read())
            
            now = time.time()
            dt = now - self.last_net_time
            
            if dt > 0.5:
                rx_spd = (rx - self.last_rx) / dt / 1024 
                tx_spd = (tx - self.last_tx) / dt / 1024 
                self.net_stats = {"rx_kbps": round(rx_spd, 1), "tx_kbps": round(tx_spd, 1)}
                self.last_rx = rx
                self.last_tx = tx
                self.last_net_time = now
        except: 
            self.net_stats = {"rx_kbps": 0, "tx_kbps": 0}

    def get_quick_stats(self):
        stats = {}
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                stats["cpu_temp"] = round(float(f.read()) / 1000.0, 1)
        except: stats["cpu_temp"] = 0

        try:
            stats["cpu_load"] = round(os.getloadavg()[0] / os.cpu_count() * 100, 1)
        except: stats["cpu_load"] = 0
        
        self.update_network_stats()
        stats.update(self.net_stats)
        return stats

    def get_slow_stats(self):
        # Return cached if too soon (prevents slamming the OS)
        if time.time() - self.last_slow_update < 2.0:
            return self.cached_stats
        
        stats = self.cached_stats.copy()
        
        try:
            res = subprocess.check_output(["vcgencmd", "measure_volts", "core"], stderr=subprocess.DEVNULL)
            stats["cpu_volts"] = float(res.decode().split("=")[1].replace("V\n", ""))
            
            res = subprocess.check_output(["vcgencmd", "measure_clock", "arm"], stderr=subprocess.DEVNULL)
            stats["cpu_clock"] = int(int(res.decode().split("=")[1]) / 1000000)
            
            res = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL)
            raw_hex = res.decode().split("=")[1].strip()
            stats["throttle_hex"] = raw_hex
            stats["throttle_flags"] = self.decode_throttle(raw_hex)

            res = subprocess.check_output(["vcgencmd", "get_mem", "gpu"], stderr=subprocess.DEVNULL)
            stats["gpu_mem"] = res.decode().split("=")[1].strip()
        except: pass

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

        try:
            st = os.statvfs('/')
            total = st.f_blocks * st.f_frsize
            free = st.f_bavail * st.f_frsize
            stats["disk_total_gb"] = round(total / (1024**3), 1)
            stats["disk_used_gb"] = round((total - free) / (1024**3), 1)
            stats["disk_percent"] = round(((total - free) / total) * 100, 1)
        except: pass

        try:
            with open("/proc/uptime", "r") as f:
                stats["uptime_sys"] = int(float(f.read().split()[0]))
        except: pass
        stats["uptime_script"] = int(time.time() - self.start_time)

        try:
            res = subprocess.check_output(["hostname", "-I"], stderr=subprocess.DEVNULL)
            stats["ip_addr"] = res.decode().strip().split(" ")[0]
        except: pass

        try:
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
                if len(lines) > 2:
                    parts = lines[2].split()
                    stats["wifi_link"] = int(float(parts[2]))
                    stats["wifi_dbm"] = int(float(parts[3]))
                    if len(parts) > 8: stats["wifi_retry"] = int(parts[8])
                    if len(parts) > 10: stats["wifi_missed"] = int(parts[10])
        except: pass

        try:
            cmd = ["ps", "-Ao", "pid,comm,pcpu,pmem", "--sort=-pcpu"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode()
            lines = output.strip().split('\n')
            procs = []
            for line in lines[1:]:
                parts = line.split()
                if len(parts) >= 4:
                    name = parts[1]
                    if name in ["ps", "sh", "head", "awk", "python"]: continue
                    procs.append({"pid": parts[0], "name": name, "cpu": parts[2], "mem": parts[3]})
                    if len(procs) >= 5: break
            stats["processes"] = procs
        except: pass

        self.cached_stats = stats
        self.last_slow_update = time.time()
        return stats

sys_mon = SystemMonitor()

# --- HELPER FUNCTIONS ---
def calculate_attitude(accel, mag):
    ax, ay, az = accel['x'], accel['y'], accel['z']
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx, my, mz = mag['x'], mag['y'], mag['z']
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    mx_comp = mx * cos_p + mz * sin_p
    my_comp = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p
    yaw = math.atan2(-my_comp, mx_comp)
    return {
        "roll": math.degrees(roll),
        "pitch": math.degrees(pitch),
        "yaw": (math.degrees(yaw) + 360) % 360
    }

CURRENT_RES = (1920, 1080)

def configure_camera(width, height):
    global CURRENT_RES
    try:
        try: cam.picam2.stop()
        except: pass
        config = cam.picam2.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
        cam.picam2.configure(config)
        cam.picam2.start()
        CURRENT_RES = (width, height)
    except Exception as e: print(f"[Cam] Config Error: {e}")

configure_camera(1920, 1080)

# --- GLOBAL STATE FOR JERK CALC ---
last_accel = {"x":0, "y":0, "z":0}
last_imu_time = time.time()

def assemble_telemetry():
    """Generates the full telemetry packet for BT and SSE"""
    global last_accel, last_imu_time
    
    # 1. Gather Data
    data = imu.get_data() if imu else {"status": "no_imu"}
    data["status"] = "ONLINE"
    
    if data["status"] == "ONLINE" and "accel" in data:
        att = calculate_attitude(data["accel"], data["mag"])
        data["attitude"] = att
        ax, ay, az = data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
        data["total_g"] = round(math.sqrt(ax**2 + ay**2 + az**2) / 9.81, 2)
        
        now = time.time()
        dt = now - last_imu_time
        if dt > 0:
            dj_x = (ax - last_accel["x"]) / dt
            dj_y = (ay - last_accel["y"]) / dt
            dj_z = (az - last_accel["z"]) / dt
            data["jerk"] = round(math.sqrt(dj_x**2 + dj_y**2 + dj_z**2), 1)
        else:
            data["jerk"] = 0.0
        
        last_accel = {"x":ax, "y":ay, "z":az}
        last_imu_time = now

    # 2. System Data
    sys_data = sys_mon.get_quick_stats()
    sys_data.update(sys_mon.get_slow_stats())
    sys_data["active_threads"] = threading.active_count()
    if data.get("temp"): sys_data["imu_temp"] = round(data["temp"], 1)
    data["sys"] = sys_data
    
    # 3. Camera Metadata
    try:
        meta = cam.picam2.capture_metadata()
        if meta:
            frame_dur = meta.get("FrameDuration", 33333)
            fps = 1000000.0 / frame_dur if frame_dur > 0 else 0
            gains = meta.get("ColourGains", (0.0, 0.0))
            data["cam_meta"] = {
                "res": f"{CURRENT_RES[0]}x{CURRENT_RES[1]}",
                "fmt": "RGB888",
                "fps": round(fps, 1),
                "exp_ms": round(meta.get("ExposureTime", 0) / 1000.0, 2),
                "a_gain": round(meta.get("AnalogueGain", 1.0), 2),
                "d_gain": round(meta.get("DigitalGain", 1.0), 2),
                "temp_k": int(meta.get("ColourTemperature", 0)),
                "lens": round(meta.get("LensPosition", 0.0), 2),
                "awb_r": round(gains[0], 2),
                "awb_b": round(gains[1], 2),
                "af_state": int(meta.get("AfState", 0))
            }
        else: data["cam_meta"] = {"res": "WAITING"}
    except Exception as e: 
        data["cam_meta"] = {"res": "ERR", "error": str(e)}
        
    return data

app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        self.last_cam_activity = 0
        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        try:
            cmd = ["hcitool", "rssi", mac]
            return int(subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode().split(":")[1])
        except: return 0

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
                    # 1. Camera Heartbeat
                    if time.time() - self.last_cam_activity > 0.2:
                        try:
                            with cam_lock: cam.picam2.capture_file(io.BytesIO(), format="jpeg")
                            self.last_cam_activity = time.time()
                        except: pass

                    # 2. Get All Data
                    data = assemble_telemetry()
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    # 3. Send
                    msg = json.dumps(data).encode('utf-8')
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    try: client.sendall(header + msg)
                    except: break
                    
                    time.sleep(0.05) # 20Hz BT Update
            except: time.sleep(1)

    def listen_commands(self, sock):
        while True:
            try:
                header = sock.recv(8)
                if not header: break
                _, length = struct.unpack("!4sI", header)
                payload = sock.recv(length)
            except: break

@app.route('/stream')
def stream():
    def generate():
        stream = io.BytesIO()
        while True:
            with cam_lock: 
                cam.picam2.capture_file(stream, format="jpeg")
                if 'node' in globals(): node.last_cam_activity = time.time()
            stream.seek(0)
            frame = stream.read()
            stream.seek(0)
            stream.truncate()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot', methods=['POST'])
def snapshot():
    stream = io.BytesIO()
    with cam_lock:
        configure_camera(4608, 2592)
        cam.picam2.capture_file(stream, format="jpeg")
        configure_camera(1920, 1080)
        if 'node' in globals(): node.last_cam_activity = time.time()
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
