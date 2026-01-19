import socket
import threading
import struct
import json
import time
import os
import requests
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, send_from_directory

# --- CONFIG ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  # CHANGE THIS to your Pi's MAC
PI_WIFI_IP = "192.168.1.183"     # CHANGE THIS to your Pi's IP
PI_VIDEO_PORT = 8000

# Robust Path Handling
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

# Initialize Flask with specific folder paths if needed, or defaults
app = Flask(__name__, template_folder="web/templates", static_folder="web/static")

# --- FLASK CONFIG ---
# CRITICAL: Disable buffering to ensure real-time telemetry flow
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

# --- BLUETOOTH CLIENT THREAD ---
def bt_client_thread():
    global bt_sock, telemetry_data
    while True:
        try:
            print(f"[GS] Connecting to Pi Bluetooth ({PI_BT_MAC})...")
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.connect((PI_BT_MAC, 1))
            
            with data_lock: 
                bt_sock = s
                telemetry_data["status"] = "ONLINE"
            
            print("[GS] Bluetooth Connected!")
            
            while True:
                header = s.recv(8)
                if not header: break
                
                type_bytes, length = struct.unpack("!4sI", header)
                
                payload = b''
                while len(payload) < length:
                    chunk = s.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                try:
                    new_data = json.loads(payload.decode('utf-8'))
                    new_data["status"] = "ONLINE"
                    
                    # --- GROUND STATION MATH: Input Voltage Estimation ---
                    # We calculate this here, effectively "offloading" logic from the Pi Node
                    if new_data.get('power', {}).get('plugged'):
                        bat_v = new_data['power'].get('voltage', 0)
                        # Heuristic: Input is usually ~10-15% higher than battery during charge + overhead
                        # This estimates the USB input voltage based on the battery state
                        est_input = round((bat_v * 1.12) + 0.15, 2)
                        new_data['power']['input_est'] = est_input
                    else:
                        new_data['power']['input_est'] = 0.0
                    
                    with data_lock: telemetry_data = new_data
                except json.JSONDecodeError:
                    pass
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with data_lock: telemetry_data["status"] = "BT LOST"
            if bt_sock: 
                try: bt_sock.close()
                except: pass
                bt_sock = None
            time.sleep(2)

threading.Thread(target=bt_client_thread, daemon=True).start()

# --- FLASK ROUTES ---

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/telemetry')
def api_telemetry():
    """JSON API for Polling (Restored)"""
    with data_lock:
        return jsonify(telemetry_data)

@app.route('/telemetry_stream')
def telemetry_stream():
    """SSE Bridge: Pushes BT data to Browser"""
    def event_stream():
        while True:
            with data_lock:
                json_data = json.dumps(telemetry_data)
            yield f"data: {json_data}\n\n"
            time.sleep(0.25) # 4Hz

    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/stream')
def proxy_video():
    """Proxy MJPEG from Pi"""
    try:
        req = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream", stream=True, timeout=2)
        return Response(stream_with_context(req.iter_content(chunk_size=4096)), content_type=req.headers['Content-Type'])
    except: return "No Signal", 404

@app.route('/snapshot', methods=['POST'])
def proxy_snapshot():
    """Trigger snapshot on Pi, save locally on GS"""
    try:
        # 1. Extract params from Dashboard request (e.g. ?exposure=1.5)
        exposure = request.args.get('exposure', 0.0)
        
        # 2. Forward to Pi with params
        # Timeout increased to 20s to allow for long exposures + processing
        resp = requests.post(
            f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot", 
            params={'exposure': exposure},
            timeout=20 
        )
        
        if resp.status_code == 200:
            timestamp = int(time.time())
            base_fn = f"snap_{timestamp}"
            
            # 1. Save Image
            img_fn = f"{base_fn}.jpg"
            with open(os.path.join(CAPTURE_DIR, img_fn), 'wb') as f:
                f.write(resp.content)
            
            # 2. Extract Headers & Save JSON Metadata (For Splatting)
            pose_str = resp.headers.get('X-Pose', '1,0,0,0')
            pose = [float(x) for x in pose_str.split(',')]
            
            accel_str = resp.headers.get('X-Accel', '0,0,0')
            accel_list = [float(x) for x in accel_str.split(',')]
            accel = {"x": accel_list[0], "y": accel_list[1], "z": accel_list[2]}
            
            metadata = {
                "image": img_fn,
                "timestamp": timestamp,
                "exposure_sec": float(exposure),
                "pose_quaternion_wxyz": pose,
                "accel_ms2": accel
            }
            
            with open(os.path.join(CAPTURE_DIR, f"{base_fn}.json"), 'w') as f:
                json.dump(metadata, f, indent=4)

            print(f"[GS] Saved {img_fn} + Metadata")
            
            # Return pose to UI so it can draw the ghost
            return jsonify({
                "status": "success", 
                "file": img_fn,
                "pose": pose 
            })
            
    except Exception as e:
        print(f"[GS] Snapshot Failed: {e}")
    return jsonify({"status": "error"}), 500

@app.route('/api/captures')
def list_captures():
    """List images for the gallery"""
    try:
        # Case-insensitive extension check
        files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.lower().endswith(('.jpg', '.jpeg'))], reverse=True)
        return jsonify(files)
    except Exception as e:
        print(f"[GS] List Error: {e}")
        return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

# --- HARDWARE CONTROL PROXIES ---

@app.route('/api/clock', methods=['POST'])
def proxy_clock():
    """Proxy clock/governor settings to Pi"""
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/clock", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 502

@app.route('/api/i2c', methods=['POST'])
def proxy_i2c():
    """Proxy I2C speed settings to Pi"""
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/i2c", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 502

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
