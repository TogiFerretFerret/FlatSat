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
from flask import Flask, Response, send_file, request, make_response, jsonify

# --- HARDWARE IMPORTS ---
try:
    from hardware.camera import CameraSystem
    from hardware.imu import IMU
    from hardware.power import PiSugar
except ImportError:
    print("Hardware libraries missing. Ensure hardware/ folder exists.")
    sys.exit(1)

# --- CONFIG ---
HTTP_PORT = 8000 
STREAM_RES = (640, 480)     
CAPTURE_RES = (2592, 1944)  

cam = CameraSystem()
imu = IMU()
power = PiSugar()
cam_lock = threading.Lock()

class SystemMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.cached_stats = {
            "cpu_volts": 0.0, "cpu_clock": 0, "throttle_hex": "0x0", "throttle_flags": ["INIT"],
            "gpu_mem": "0M", "ram_total_mb": 0, "ram_used_mb": 0, "ram_percent": 0.0,
            "disk_total_gb": 0.0, "disk_used_gb": 0.0, "disk_percent": 0.0,
            "uptime_sys": 0, "uptime_script": 0, "ip_addr": "Unknown",
            "wifi_dbm": 0, "wifi_link": 0, "wifi_retry": 0, "wifi_missed": 0, "processes": []
        }
        self.last_slow_update = 0
        self.start_time = time.time()
        self.last_net_time = time.time()
        self.last_rx = 0
        self.last_tx = 0
        self.net_stats = {"rx_kbps": 0.0, "tx_kbps": 0.0}

    def update_network_stats(self):
        try:
            iface = "wlan0" if os.path.exists("/sys/class/net/wlan0") else "eth0"
            rx = int(open(f"/sys/class/net/{iface}/statistics/rx_bytes").read())
            tx = int(open(f"/sys/class/net/{iface}/statistics/tx_bytes").read())
            now = time.time()
            dt = now - self.last_net_time
            if dt > 0.5:
                self.net_stats = {"rx_kbps": round((rx-self.last_rx)/dt/1024, 1), "tx_kbps": round((tx-self.last_tx)/dt/1024, 1)}
                self.last_rx, self.last_tx, self.last_net_time = rx, tx, now
        except: pass

    def get_stats(self):
        with self.lock:
            try:
                self.update_network_stats()
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    cpu_temp = round(float(f.read()) / 1000.0, 1)
                cpu_load = round(os.getloadavg()[0] / os.cpu_count() * 100, 1)
            except: cpu_temp, cpu_load = 0, 0

            if time.time() - self.last_slow_update > 2.0:
                self._update_slow_stats()
                self.last_slow_update = time.time()

            full = self.cached_stats.copy()
            full.update(self.net_stats)
            full["cpu_temp"], full["cpu_load"] = cpu_temp, cpu_load
            return full

    def _update_slow_stats(self):
        try:
            self.cached_stats["cpu_volts"] = float(subprocess.check_output(["vcgencmd", "measure_volts", "core"], stderr=subprocess.DEVNULL).decode().split("=")[1].replace("V\n", ""))
            self.cached_stats["cpu_clock"] = int(int(subprocess.check_output(["vcgencmd", "measure_clock", "arm"], stderr=subprocess.DEVNULL).decode().split("=")[1]) / 1000000)
            self.cached_stats["throttle_hex"] = subprocess.check_output(["vcgencmd", "get_throttled"], stderr=subprocess.DEVNULL).decode().split("=")[1].strip()
        except: pass 

        try:
            mem = {l.split(':')[0]: int(l.split()[1]) for l in open("/proc/meminfo")}
            total, avail = mem.get("MemTotal", 1), mem.get("MemAvailable", 0)
            self.cached_stats["ram_total_mb"] = int(total/1024)
            self.cached_stats["ram_used_mb"] = int((total-avail)/1024)
            self.cached_stats["ram_percent"] = round(((total-avail)/total)*100, 1)
        except: pass

        try:
            total, used, free = shutil.disk_usage("/")
            self.cached_stats["disk_total_gb"] = round(total/(1024**3), 1)
            self.cached_stats["disk_used_gb"] = round(used/(1024**3), 1)
            self.cached_stats["disk_percent"] = round((used/total)*100, 1)
        except: pass

        try:
            self.cached_stats["uptime_sys"] = int(float(open("/proc/uptime").read().split()[0]))
            self.cached_stats["uptime_script"] = int(time.time() - self.start_time)
            self.cached_stats["ip_addr"] = subprocess.check_output(["hostname", "-I"], stderr=subprocess.DEVNULL).decode().strip().split(" ")[0]
        except: pass

        try:
            if os.path.exists("/proc/net/wireless"):
                lines = open("/proc/net/wireless").readlines()
                if len(lines) > 2:
                    parts = lines[2].split()
                    if ":" in parts[0]:
                        self.cached_stats["wifi_link"] = int(float(parts[2].replace('.', '')))
                        self.cached_stats["wifi_dbm"] = int(float(parts[3].replace('.', '')))
        except: pass

        try:
            out = subprocess.check_output(["ps", "-Ao", "pid,comm,pcpu,pmem", "--sort=-pcpu"], stderr=subprocess.DEVNULL).decode()
            procs = []
            for line in out.strip().split('\n')[1:6]:
                parts = line.split()
                if len(parts) >= 4 and parts[1] not in ["ps", "sh"]:
                    procs.append({"pid": parts[0], "name": parts[1], "cpu": parts[2], "mem": parts[3]})
            self.cached_stats["processes"] = procs
        except: pass

sys_mon = SystemMonitor()

# --- HELPER FUNCTIONS ---
def calculate_attitude(accel, mag):
    ax, ay, az = accel['x'], accel['y'], accel['z']
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    mx, my, mz = mag['x'], mag['y'], mag['z']
    
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

# DEFINE THIS GLOBALLY SO SNAPSHOT CAN SEE IT
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

# Initial Config
configure_camera(STREAM_RES[0], STREAM_RES[1])

def assemble_telemetry():
    data = imu.get_data() if imu else {"status": "no_imu"}
    data["status"] = "ONLINE"
    
    if data["status"] == "ONLINE" and "accel" in data:
        data["attitude"] = calculate_attitude(data["accel"], data["mag"])
        ax, ay, az = data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]
        data["total_g"] = round(math.sqrt(ax**2 + ay**2 + az**2) / 9.81, 2)
        
        # Jerk calc omitted for brevity, add if needed
        data["jerk"] = 0.0 

    data["sys"] = sys_mon.get_stats()
    data["sys"]["active_threads"] = threading.active_count()
    if data.get("temp"): data["sys"]["imu_temp"] = round(data["temp"], 1)
    data["power"] = power.get_data()
    
    try:
        meta = cam.picam2.capture_metadata()
        if meta:
            data["cam_meta"] = {
                "res": f"{CURRENT_RES[0]}x{CURRENT_RES[1]}", "fps": 0,
                "exp_ms": round(meta.get("ExposureTime", 0)/1000.0, 2),
                "a_gain": round(meta.get("AnalogueGain", 1.0), 2),
                "d_gain": round(meta.get("DigitalGain", 1.0), 2),
                "awb_r": round(meta.get("ColourGains", (0,0))[0], 2),
                "awb_b": round(meta.get("ColourGains", (0,0))[1], 2),
                "af_state": int(meta.get("AfState", 0))
            }
    except: data["cam_meta"] = {"res": "ERR"}
    return data

app = Flask(__name__)

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

# --- API ROUTES ---
@app.route('/api/clock', methods=['POST'])
def set_clock():
    try:
        data = request.json
        mode = data.get('mode')
        speed = data.get('speed')
        if mode in ['performance', 'powersave', 'ondemand']:
            for cpu in os.listdir("/sys/devices/system/cpu"):
                if cpu.startswith("cpu") and cpu[3:].isdigit():
                    gov = os.path.join("/sys/devices/system/cpu", cpu, "cpufreq", "scaling_governor")
                    if os.path.exists(gov): open(gov, "w").write(mode)
            return jsonify({"status": "ok"})
        if speed and int(speed) > 0:
            subprocess.run(["sudo", "/usr/bin/cpufreq-set", "-u", str(speed)])
            return jsonify({"status": "ok"})
        return jsonify({"status": "error"}), 400
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 500

@app.route('/api/i2c', methods=['POST'])
def set_i2c():
    try:
        baud = request.json.get('baudrate')
        if baud:
            subprocess.run(["sudo", "/usr/bin/dtparam", f"i2c_arm_baudrate={baud}"], check=True)
            return jsonify({"status": "ok"})
        return jsonify({"status": "error"}), 400
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 500

@app.route('/api/power', methods=['POST'])
def set_power():
    try:
        action = request.json.get('action') 
        val = request.json.get('value')
        
        if action == 'shutdown':
            delay = int(val) if val else 10
            if power.shutdown(delay):
                subprocess.Popen(f"sleep {delay} && sudo /sbin/shutdown -h now", shell=True)
                return jsonify({"status": "ok", "msg": f"Battery Off in {delay}s"})
            return jsonify({"status": "error", "msg": "Hardware write failed"})
            
        elif action == 'cycle':
            if power.power_cycle(20):
                subprocess.Popen("sleep 1 && sudo /sbin/shutdown -h now", shell=True)
                return jsonify({"status": "ok", "msg": "Cycling Power..."})
            return jsonify({"status": "error", "msg": "Watchdog enable failed"})
            
        elif action == 'reboot':
            subprocess.Popen("sleep 1 && sudo /sbin/reboot", shell=True)
            return jsonify({"status": "ok", "msg": "Rebooting..."})
            
        elif action == 'charging':
            enable = bool(val)
            if power.set_charging(enable):
                return jsonify({"status": "ok", "charging": enable})
            return jsonify({"status": "error", "msg": "Hardware write failed"})
            
        return jsonify({"status": "error", "msg": "Invalid action"}), 400
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 500

class HybridNode:
    def __init__(self):
        self.running = True
        self.last_cam_activity = 0
        threading.Thread(target=self.run_bt_server, daemon=True).start()

    def get_rssi(self, mac):
        try: return int(subprocess.check_output(["/usr/bin/hcitool", "rssi", mac], stderr=subprocess.DEVNULL).decode().split(":")[1])
        except: return 0

    def run_bt_server(self):
        print("[BT] Init...")
        os.system("sudo /usr/bin/hciconfig hci0 piscan")
        os.system("/usr/bin/sdptool add --channel=1 SP")
        
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.bind(("00:00:00:00:00:00", 1))
        s.listen(1)
        
        while self.running:
            try:
                client, addr = s.accept()
                print(f"[BT] Connected: {addr}")
                while self.running:
                    if time.time() - self.last_cam_activity > 0.5:
                        try:
                            with cam_lock: cam.picam2.capture_file(io.BytesIO(), format="jpeg")
                            self.last_cam_activity = time.time()
                        except: pass
                    data = assemble_telemetry()
                    data["rssi"] = self.get_rssi(addr[0])
                    msg = json.dumps(data).encode('utf-8')
                    client.sendall(struct.pack("!4sI", b"TELE", len(msg)) + msg)
                    time.sleep(0.2)
            except: time.sleep(1)

@app.route('/stream')
def stream():
    def gen():
        s = io.BytesIO()
        while True:
            try:
                #cam.picam2.capture_file(s, format="jpeg")
                if 'node' in globals(): node.last_cam_activity = time.time()
                s.seek(0)
                frame = s.read()
                s.seek(0); s.truncate()
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception: # FIX: Explicitly catch Exception to ignore GeneratorExit
                pass
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot', methods=['POST'])
def snapshot():
    s = io.BytesIO()
    exp = request.args.get('exposure', 0.0, type=float)
    with cam_lock:
        data = assemble_telemetry()
        # FIX: Ensure configure_camera is defined/available
        configure_camera(CAPTURE_RES[0], CAPTURE_RES[1])
        if exp > 0:
            us = int(exp*1000000)
            cam.picam2.set_controls({"FrameDurationLimits": (us+10000, us+10000), "ExposureTime": us})
            time.sleep(exp + 0.5)
        else: time.sleep(0.5)
        cam.picam2.capture_file(s, format="jpeg")
        configure_camera(STREAM_RES[0], STREAM_RES[1])
        cam.picam2.set_controls({"ExposureTime": 0, "FrameDurationLimits": (0, 0)})
        if 'node' in globals(): node.last_cam_activity = time.time()
    s.seek(0)
    resp = make_response(send_file(s, mimetype='image/jpeg', download_name='snap.jpg'))
    resp.headers['X-Pose'] = ",".join(map(str, data.get("quaternion", [1,0,0,0])))
    resp.headers['X-Accel'] = f"{data.get('accel',{'x':0})['x']},{data.get('accel',{'y':0})['y']},{data.get('accel',{'z':0})['z']}"
    return resp

if __name__ == "__main__":
    node = HybridNode()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
