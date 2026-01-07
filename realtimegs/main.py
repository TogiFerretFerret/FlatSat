import socket
import threading
import time
import json
import struct
import os
import io
import sys
import subprocess
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

# Startup at 1080p
configure_camera(1920, 1080)

app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        
        # State Cache (Prevents slow subprocess calls from blocking the loop)
        self.cached_stats = {
            "cpu_temp": 0, "cpu_load": 0, "mem_percent": 0, 
            "disk_percent": 0, "uptime": 0, "wifi_signal": 0, 
            "throttled": "0x0"
        }
        self.last_stats_time = 0
        
        self.cached_rssi = 0
        self.last_rssi_time = 0

        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        """Get Bluetooth Signal Strength (Cached every 2s)"""
        if time.time() - self.last_rssi_time < 2.0:
            return self.cached_rssi
            
        try:
            cmd = ["hcitool", "rssi", mac]
            res = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            val = int(res.decode().split(":")[1])
            self.cached_rssi = val
            self.last_rssi_time = time.time()
            return val
        except:
            return self.cached_rssi

    def get_system_stats(self):
        """Gather CPU, RAM, Disk, Uptime, WiFi, and Throttling (Cached every 2s)"""
        if time.time() - self.last_stats_time < 2.0:
            return self.cached_stats

        stats = self.cached_stats.copy()
        try:
            # 1. CPU Temp
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                stats["cpu_temp"] = float(f.read()) / 1000.0
            
            # 2. CPU Load
            cpu_count = os.cpu_count() or 1
            stats["cpu_load"] = round(os.getloadavg()[0] * 100 / cpu_count, 1)
            
            # 3. Memory
            with open("/proc/meminfo", "r") as f:
                lines = f.readlines()
                total = int(lines[0].split()[1])
                available = int(lines[2].split()[1])
                stats["mem_percent"] = round((1 - available/total) * 100, 1)

            # 4. Disk Usage
            st = os.statvfs('/')
            total = st.f_blocks * st.f_frsize
            free = st.f_bavail * st.f_frsize
            stats["disk_percent"] = round((1 - free/total) * 100, 1)
            
            # 5. Uptime (New)
            with open("/proc/uptime", "r") as f:
                stats["uptime"] = int(float(f.read().split()[0]))

            # 6. Wi-Fi Signal (New)
            with open("/proc/net/wireless", "r") as f:
                for line in f:
                    if "wlan" in line:
                        # Field 3 represents level in dBm usually (e.g. -67.)
                        stats["wifi_signal"] = int(float(line.split()[3]))
                        break

            # 7. Throttling Status (New)
            # Checks for under-voltage or overheating flags
            res = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL)
            stats["throttled"] = res.decode().strip().split('=')[1]
            
        except Exception:
            pass
        
        self.cached_stats = stats
        self.last_stats_time = time.time()
        return stats

    def run_bt_server(self):
        print("[BT] Configuring Adapter...")
        os.system("sudo hciconfig hci0 piscan")
        os.system("sdptool add SP")

        try:
            server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            server_sock.bind(("00:00:00:00:00:00", 1))
            server_sock.listen(1)
            print("[BT] Ready...")

            while self.running:
                try:
                    client, addr = server_sock.accept()
                    print(f"[BT] Connected: {addr}")
                    while self.running:
                        # 1. Base Sensor Data
                        data = imu.get_data() if imu else {"status": "no_imu"}
                        
                        # 2. Add System Stats & RSSI
                        sys_stats = self.get_system_stats()
                        data.update(sys_stats)
                        data["rssi"] = self.get_rssi(addr[0])
                        
                        # 3. Send
                        msg = json.dumps(data).encode('utf-8')
                        header = struct.pack("!4sI", b"TELE", len(msg))
                        client.sendall(header + msg)
                        time.sleep(0.05)
                except Exception as e:
                    print(f"[BT] Disconnected: {e}")
                    if client:
                        try: 
                            client.close()
                        except: 
                            pass
                    time.sleep(1)
        except Exception as e:
            print(f"[BT] Server Error: {e}")

# --- FLASK ROUTES (Unchanged) ---
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
