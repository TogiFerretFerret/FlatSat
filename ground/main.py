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

# --- CONFIG ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  # CHANGE THIS to your Pi's MAC
try:
    # Tries to resolve 'cubesat.local' dynamically
    PI_WIFI_IP = socket.gethostbyname("cubesat.local")
    print(f"Resolved cubesat.local to: {PI_WIFI_IP}")
except socket.gaierror:
    # Fallback if the Pi is offline or mDNS fails
    print("Could not resolve cubesat.local! Using fallback IP.")
    PI_WIFI_IP = "192.168.1.183"
PI_VIDEO_PORT = 8000

# Robust Path Handling
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

app = Flask(__name__, template_folder="web/templates", static_folder="web/static")

# --- MACOS SUPPORT UTILS ---
class SerialSocketAdapter:
    def __init__(self, serial_port):
        self.ser = serial_port
    
    def recv(self, bufsize):
        return self.ser.read(bufsize)
        
    def sendall(self, data):
        self.ser.write(data)
        
    def close(self):
        self.ser.close()

def find_macos_port():
    patterns = ["/dev/cu.*pi*", "/dev/cu.*Serial*", "/dev/cu.*Artemis*"]
    candidates = []
    for p in patterns:
        candidates.extend(glob.glob(p))
    candidates = [c for c in candidates if "Bluetooth-Incoming" not in c and "wlan" not in c]
    if candidates: return candidates[0]
    return None

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Cache-Control', 'no-cache')
    response.headers.add('X-Accel-Buffering', 'no')
    return response

# Global State
telemetry_data = {"status": "CONNECTING...", "sys": {}, "cam_meta": {}}
bt_sock = None
data_lock = threading.Lock()

def bt_client_thread():
    global bt_sock, telemetry_data
    if sys.platform == "darwin":
        try: import serial
        except ImportError:
            print("CRITICAL: pyserial missing. Run: uv pip install pyserial")
            return

    while True:
        try:
            s = None
            if sys.platform == "darwin":
                port = find_macos_port()
                if port:
                    print(f"[GS] Connecting to macOS Serial: {port}...")
                    ser = serial.Serial(port, 115200, timeout=1) 
                    s = SerialSocketAdapter(ser)
                else:
                    time.sleep(2); continue
            else:
                s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                s.connect((PI_BT_MAC, 1))
            
            with data_lock: 
                bt_sock = s
                telemetry_data["status"] = "ONLINE"
            
            while True:
                header = s.recv(8)
                if not header or len(header) < 8: 
                    if len(header) == 0: break 
                    continue 
                
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
                        bat_v = new_data['power'].get('voltage', 0)
                        est_input = round((bat_v * 1.12) + 0.15, 2)
                        new_data['power']['input_est'] = est_input
                    else:
                        if 'power' in new_data: new_data['power']['input_est'] = 0.0
                    with data_lock: telemetry_data = new_data
                except json.JSONDecodeError: pass
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with data_lock: telemetry_data["status"] = "BT LOST"
            if bt_sock: 
                try: bt_sock.close()
                except: pass
                bt_sock = None
            time.sleep(2)

threading.Thread(target=bt_client_thread, daemon=True).start()

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/telemetry')
def api_telemetry():
    with data_lock: return jsonify(telemetry_data)

@app.route('/telemetry_stream')
def telemetry_stream():
    def event_stream():
        while True:
            with data_lock: json_data = json.dumps(telemetry_data)
            yield f"data: {json_data}\n\n"
            time.sleep(0.25)
    return Response(event_stream(), mimetype='text/event-stream')

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
    except Exception as e: print(f"[GS] Snapshot Failed: {e}")
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
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/clock", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

@app.route('/api/i2c', methods=['POST'])
def proxy_i2c():
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/i2c", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

@app.route('/api/power', methods=['POST'])
def proxy_power():
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/power", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
