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
import shutil
from flask import Flask, Response, send_file, request

# --- HARDWARE IMPORTS ---
try:
    from hardware.camera import CameraSystem
    from hardware.imu import IMU
except ImportError:
    print("Hardware libraries missing. Ensure hardware/ folder exists.")
    sys.exit(1)

# --- CONFIG ---
HTTP_PORT = 8000 
STREAM_RES = (640, 480)     # Fast, low lag
CAPTURE_RES = (2592, 1944)  # 5MP High Quality

# Initialize Global Hardware
cam = CameraSystem()
imu = IMU()
cam_lock = threading.Lock()

# --- SYSTEM MONITOR ---
class SystemMonitor:
    def __init__(self):
        print("[SysMon] Initializing System Monitor...")
        self.lock = threading.Lock()
        
        # FULL DEFAULTS (The "Motherflippin Data" Foundation)
        self.cached_stats = {
            "cpu_volts": 0.0, "cpu_clock": 0, "throttle_hex": "0x0", "throttle_flags": ["INIT"],
            "gpu_mem": "0M", 
            "ram_total_mb": 0, "ram_used_mb": 0, "ram_percent": 0.0,
            "disk_total_gb": 0.0, "disk_used_gb": 0.0, "disk_percent": 0.0,
            "uptime_sys": 0, "uptime_script": 0, "ip_addr": "Unknown",
            "wifi_dbm": 0, "wifi_link": 0, "wifi_retry": 0, "wifi_missed": 0,
            "processes": []
        }
        self.last_slow_update = 0
        self.start_time = time.time()
        self.last_net_time = time.time()
        self.last_rx = 0
        self.last_tx = 0
        self.net_stats = {"rx_kbps": 0.0, "tx_kbps": 0.0}

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
            if not os.path.exists(f"/sys/class/net/{iface}"): iface = "eth0"
            
            rx_path = f"/sys/class/net/{iface}/statistics/rx_bytes"
            tx_path = f"/sys/class/net/{iface}/statistics/tx_bytes"
            
            if os.path.exists(rx_path):
                with open(rx_path, "r") as f: rx = int(f.read())
                with open(tx_path, "r") as f: tx = int(f.read())
                
                now = time.time()
                dt = now - self.last_net_time
                if dt > 0.5:
                    self.net_stats = {
                        "rx_kbps": round((rx - self.last_rx) / dt / 1024, 1),
                        "tx_kbps": round((tx - self.last_tx) / dt / 1024, 1)
                    }
                    self.last_rx = rx
                    self.last_tx = tx
                    self.last_net_time = now
        except: pass

    def get_stats(self):
        """Returns the COMPLETE blended stats payload safely."""
        with self.lock:
            # 1. Update Fast Stats (Network/CPU Load)
            try:
                self.update_network_stats()
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    cpu_temp = round(float(f.read()) / 1000.0, 1)
                cpu_load = round(os.getloadavg()[0] / os.cpu_count() * 100, 1)
            except: 
                cpu_temp = 0
                cpu_load = 0

            # 2. Update Slow Stats (Hardware/Disk) every 2s
            if time.time() - self.last_slow_update > 2.0:
                self._update_slow_stats()
                self.last_slow_update = time.time()

            # 3. Merge
            full_stats = self.cached_stats.copy()
            full_stats.update(self.net_stats)
            full_stats["cpu_temp"] = cpu_temp
            full_stats["cpu_load"] = cpu_load
            return full_stats

    def _update_slow_stats(self):
        # Internal helper - runs inside lock
        try:
            res = subprocess.check_output(["vcgencmd", "measure_volts", "core"], stderr=subprocess.DEVNULL)
            self.cached_stats["cpu_volts"] = float(res.decode().split("=")[1].replace("V\n", ""))
            
            res = subprocess.check_output(["vcgencmd", "measure_clock", "arm"], stderr=subprocess.DEVNULL)
            self.cached_stats["cpu_clock"] = int(int(res.decode().split("=")[1]) / 1000000)
            
            res = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL)
            raw_hex = res.decode().split("=")[1].strip()
            self.cached_stats["throttle_hex"] = raw_hex
            self.cached_stats["throttle_flags"] = self.decode_throttle(raw_hex)
        except: pass # Non-Pi hardware support

        try:
            with open("/proc/meminfo", "r") as f:
                mem = {line.split(':')[0]: int(line.split()[1]) for line in f}
            total = mem.get("MemTotal", 1)
            avail = mem.get("MemAvailable", 0)
            self.cached_stats["ram_total_mb"] = int(total / 1024)
            self.cached_stats["ram_used_mb"] = int((total - avail) / 1024)
            self.cached_stats["ram_percent"] = round(((total - avail) / total) * 100, 1)
        except: pass

        try:
            total, used, free = shutil.disk_usage("/")
            self.cached_stats["disk_total_gb"] = round(total / (1024**3), 1)
            self.cached_stats["disk_used_gb"] = round(used / (1024**3), 1)
            self.cached_stats["disk_percent"] = round((used / total) * 100, 1)
        except: pass

        try:
            with open("/proc/uptime", "r") as f:
                self.cached_stats["uptime_sys"] = int(float(f.read().split()[0]))
            self.cached_stats["uptime_script"] = int(time.time() - self.start_time)
            
            res = subprocess.check_output(["hostname", "-I"], stderr=subprocess.DEVNULL)
            self.cached_stats["ip_addr"] = res.decode().strip().split(" ")[0]
        except: pass

        try:
            if os.path.exists("/proc/net/wireless"):
                with open("/proc/net/wireless", "r") as f:
                    lines = f.readlines()
                    if len(lines) > 2:
                        parts = lines[2].split()
                        if ":" in parts[0]:
                            self.cached_stats["wifi_link"] = int(float(parts[2]))
                            self.cached_stats["wifi_dbm"] = int(float(parts[3]))
        except: pass

sys_mon = SystemMonitor()

# --- HELPER FUNCTIONS ---
def calculate_attitude(accel, mag):
    ax, ay, az = accel['x'], accel['y'], accel['z']
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx, my, mz = mag['x'], mag['y'], mag['z']
    # Tilt compensation
    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx_comp = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    my_comp = mx * math.sin(roll_rad) * math.sin(pitch_rad) + my * math.cos(roll_rad) - mz * math.sin(roll_rad) * math.cos(pitch_rad)
    
    yaw = math.atan2(-my_comp, mx_comp)
    return {
        "roll": math.degrees(roll),
        "pitch": math.degrees(pitch),
        "yaw": (math.degrees(yaw) + 360) % 360
    }

CURRENT_RES = STREAM_RES

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

configure_camera(STREAM_RES[0], STREAM_RES[1])

# --- TELEMETRY ASSEMBLY ---
last_accel = {"x":0, "y":0, "z":0}
last_imu_time = time.time()
frame_counter = 0

def assemble_telemetry():
    global last_accel, last_imu_time, frame_counter
    frame_counter += 1
    
    # 1. IMU Data
    data = imu.get_data() if imu else {"status": "no_imu"}
    data["status"] = "ONLINE"
    
    if data["status"] == "ONLINE" and "accel" in data:
        data["attitude"] = calculate_attitude(data["accel"], data["mag"])
        ax, ay, az = data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
        data["total_g"] = round(math.sqrt(ax**2 + ay**2 + az**2) / 9.81, 2)
        
        # Jerk Calc
        now = time.time()
        dt = now - last_imu_time
        if dt > 0:
            dj = math.sqrt((ax - last_accel["x"])**2 + (ay - last_accel["y"])**2 + (az - last_accel["z"])**2)
            data["jerk"] = round(dj / dt, 1)
        else: data["jerk"] = 0.0
        last_accel = {"x":ax, "y":ay, "z":az}
        last_imu_time = now

    # 2. System Data (The Critical Part)
    data["sys"] = sys_mon.get_stats()
    data["sys"]["active_threads"] = threading.active_count()
    if data.get("temp"): data["sys"]["imu_temp"] = round(data["temp"], 1)
    
    # Debug Print every ~4 seconds (assuming 5Hz calls)
    if frame_counter % 20 == 0:
        s = data["sys"]
        print(f"[Telem] CPU:{s['cpu_load']}% Temp:{s['cpu_temp']}C Mem:{s['ram_percent']}% WiFi:{s['wifi_dbm']}dBm")

    # 3. Camera Metadata
    try:
        meta = cam.picam2.capture_metadata()
        if meta:
            data["cam_meta"] = {
                "res": f"{CURRENT_RES[0]}x{CURRENT_RES[1]}",
                "fps": 0, # Simplify
                "exp_ms": round(meta.get("ExposureTime", 0) / 1000.0, 2),
                "a_gain": round(meta.get("AnalogueGain", 1.0), 2),
                "d_gain": round(meta.get("DigitalGain", 1.0), 2),
                "awb_r": round(meta.get("ColourGains", (0,0))[0], 2),
                "awb_b": round(meta.get("ColourGains", (0,0))[1], 2),
                "af_state": int(meta.get("AfState", 0))
            }
    except: 
        data["cam_meta"] = {"res": "ERR"}
        
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
            return int(subprocess.check_output(["hcitool", "rssi", mac], stderr=subprocess.DEVNULL).decode().split(":")[1])
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
                    # Heartbeat
                    if time.time() - self.last_cam_activity > 0.5:
                        try:
                            with cam_lock: cam.picam2.capture_file(io.BytesIO(), format="jpeg")
                            self.last_cam_activity = time.time()
                        except: pass

                    data = assemble_telemetry()
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    msg = json.dumps(data).encode('utf-8')
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    try: client.sendall(header + msg)
                    except: break
                    
                    time.sleep(0.2) # 5Hz
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
            # Lock removed to prevent blocking high-speed stream
            try:
                cam.picam2.capture_file(stream, format="jpeg")
                if 'node' in globals(): node.last_cam_activity = time.time()
                
                stream.seek(0)
                frame = stream.read()
                
                # Reset buffer to prevent memory leak
                stream.seek(0)
                stream.truncate()
                
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except:
                # Ignore frame drops during reconfiguration
                pass
            time.sleep(0.03)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry_stream')
def telemetry_stream():
    """DIRECT WiFi Stream (Restored for reliability)"""
    def event_stream():
        while True:
            data = assemble_telemetry()
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.25)
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/snapshot', methods=['POST'])
def snapshot():
    stream = io.BytesIO()
    exposure_sec = request.args.get('exposure', default=0.0, type=float)
    
    with cam_lock:
        print(f"[Cam] Snap Request (Exp: {exposure_sec}s)")
        configure_camera(CAPTURE_RES[0], CAPTURE_RES[1])
        
        if exposure_sec > 0:
            us = int(exposure_sec * 1000000)
            cam.picam2.set_controls({"FrameDurationLimits": (us+10000, us+10000), "ExposureTime": us})
            time.sleep(exposure_sec + 0.5)
        else:
            time.sleep(0.5) 
        
        cam.picam2.capture_file(stream, format="jpeg")
        
        configure_camera(STREAM_RES[0], STREAM_RES[1])
        cam.picam2.set_controls({"ExposureTime": 0, "FrameDurationLimits": (0, 0)})
        if 'node' in globals(): node.last_cam_activity = time.time()
        
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
