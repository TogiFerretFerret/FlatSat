import socket
import threading
import struct
import json
import time
import os
import requests
import sys
import glob
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, send_from_directory

# --- IMPORT YOUR ALGORITHM ---
try:
    from lunar_landing_analyzer import analyze_image
except ImportError as e:
    print(f"WARNING: lunar_landing_analyzer failed to load. Missing scipy/matplotlib? {e}")
    analyze_image = None

# --- CONFIG ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"
try:
    PI_WIFI_IP = socket.gethostbyname("cubesat.local")
except socket.gaierror:
    PI_WIFI_IP = "192.168.1.183"
PI_VIDEO_PORT = 8000

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

app = Flask(__name__, template_folder="web/templates", static_folder="web/static")

# --- MACOS SUPPORT ---
class SerialSocketAdapter:
    def __init__(self, serial_port): self.ser = serial_port
    def recv(self, bufsize): return self.ser.read(bufsize)
    def sendall(self, data): self.ser.write(data)
    def close(self): self.ser.close()

def find_macos_port():
    patterns = ["/dev/cu.*pi*", "/dev/cu.*Serial*", "/dev/cu.*Artemis*"]
    candidates = []
    for p in patterns: candidates.extend(glob.glob(p))
    candidates = [c for c in candidates if "Bluetooth-Incoming" not in c and "wlan" not in c]
    return candidates[0] if candidates else None

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Cache-Control', 'no-cache')
    response.headers.add('X-Accel-Buffering', 'no')
    return response

telemetry_data = {"status": "CONNECTING...", "sys": {}, "cam_meta": {}}
bt_sock = None
data_lock = threading.Lock()

def bt_client_thread():
    global bt_sock, telemetry_data
    if sys.platform == "darwin":
        try: import serial
        except ImportError: return
    while True:
        try:
            s = None
            if sys.platform == "darwin":
                port = find_macos_port()
                if port:
                    ser = serial.Serial(port, 115200, timeout=1)
                    s = SerialSocketAdapter(ser)
                else: time.sleep(2); continue
            else:
                s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                s.connect((PI_BT_MAC, 1))
            with data_lock:
                bt_sock = s
                telemetry_data["status"] = "ONLINE"
            while True:
                header = s.recv(8)
                if not header or len(header) < 8: break
                type_bytes, length = struct.unpack("!4sI", header)
                payload = b''
                while len(payload) < length:
                    chunk = s.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                try:
                    new_data = json.loads(payload.decode('utf-8'))
                    new_data["status"] = "ONLINE"
                    if new_data.get('power', {}).get('plugged'):
                        bat_v = float(new_data['power'].get('voltage', 0))
                        new_data['power']['input_est'] = round(bat_v + 0.25, 2)
                    with data_lock: telemetry_data = new_data
                except: pass
        except:
            with data_lock: telemetry_data["status"] = "BT LOST"
            time.sleep(2)

threading.Thread(target=bt_client_thread, daemon=True).start()

@app.route('/')
def index(): return render_template('dashboard.html')

@app.route('/api/telemetry')
def api_telemetry():
    with data_lock: return jsonify(telemetry_data)

@app.route('/api/analyze/<filename>', methods=['POST'])
def run_analysis(filename):
    """Manually triggers the lunar landing analyzer for a specific file."""
    if not analyze_image:
        return jsonify({"status": "error", "message": "Analyzer libraries not loaded"}), 500
    
    file_path = os.path.join(CAPTURE_DIR, filename)
    if not os.path.exists(file_path):
        return jsonify({"status": "error", "message": "File not found"}), 404

    try:
        # Run the SciPy-heavy algorithm
        output_path = analyze_image(file_path, output_dir=CAPTURE_DIR)
        return jsonify({"status": "success", "analysis_file": os.path.basename(output_path)})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/stream')
def proxy_video():
    try:
        req = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream", stream=True, timeout=2)
        return Response(stream_with_context(req.iter_content(chunk_size=4096)), content_type=req.headers['Content-Type'])
    except: return "No Signal", 404

@app.route('/snapshot', methods=['POST'])
def proxy_snapshot():
    try:
        exposure = request.args.get('exposure', 0.0)
        resp = requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot", params={'exposure': exposure}, timeout=20)
        if resp.status_code == 200:
            timestamp = int(time.time())
            base_fn = f"snap_{timestamp}"
            img_fn = f"{base_fn}.jpg"
            with open(os.path.join(CAPTURE_DIR, img_fn), 'wb') as f: f.write(resp.content)
            pose = [float(x) for x in resp.headers.get('X-Pose', '1,0,0,0').split(',')]
            accel = [float(x) for x in resp.headers.get('X-Accel', '0,0,0').split(',')]
            metadata = {
                "image": img_fn, "timestamp": timestamp, "exposure_sec": float(exposure),
                "pose_quaternion_wxyz": pose, "accel_ms2": {"x":accel[0], "y":accel[1], "z":accel[2]}
            }
            with open(os.path.join(CAPTURE_DIR, f"{base_fn}.json"), 'w') as f: json.dump(metadata, f, indent=4)
            return jsonify({"status": "success", "file": img_fn, "pose": pose})
    except: pass
    return jsonify({"status": "error"}), 500

@app.route('/api/captures')
def list_captures():
    try:
        files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.lower().endswith(('.jpg', '.jpeg'))], reverse=True)
        return jsonify(files)
    except: return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

@app.route('/api/clock', methods=['POST'])
def proxy_clock():
    requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/clock", json=request.json, timeout=3)
    return jsonify({"status": "proxied"})

@app.route('/api/i2c', methods=['POST'])
def proxy_i2c():
    requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/i2c", json=request.json, timeout=3)
    return jsonify({"status": "proxied"})

@app.route('/api/power', methods=['POST'])
def proxy_power():
    requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/power", json=request.json, timeout=3)
    return jsonify({"status": "proxied"})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
