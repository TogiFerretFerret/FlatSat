import socket
import threading
import struct
import json
import time
import os
import requests
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, send_from_directory

# --- CONFIGURATION ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  # <--- CHECK THIS
PI_WIFI_IP = "192.168.1.183"     # <--- CHECK THIS
PI_VIDEO_PORT = 8000

# Setup Capture Directory
CAPTURE_DIR = os.path.join(os.getcwd(), "captures")
if not os.path.exists(CAPTURE_DIR):
    os.makedirs(CAPTURE_DIR)

app = Flask(__name__)

# --- GLOBAL STATE ---
telemetry_data = {
    "status": "CONNECTING...",
    "rssi": 0,
    "pos_offset": {"x": 0, "y": 0, "z": 0},
    "tracking_active": False
}
bt_sock = None
lock = threading.Lock()

def bt_client_thread():
    """Background thread to fetch Telemetry over Bluetooth."""
    global telemetry_data, bt_sock
    
    while True:
        try:
            print(f"[GS] Connecting BT to {PI_BT_MAC}...")
            # Create fresh socket
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.connect((PI_BT_MAC, 1))
            
            with lock:
                bt_sock = s # Expose socket for sending commands
                telemetry_data["status"] = "ONLINE"
            
            print("[GS] Bluetooth Connected!")

            while True:
                # 1. Read Header
                header = s.recv(8)
                if not header: break
                type_bytes, length = struct.unpack("!4sI", header)
                
                # 2. Read Payload
                payload = b''
                while len(payload) < length:
                    chunk = s.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                if len(payload) != length: break

                # 3. Update State
                try:
                    data = json.loads(payload.decode('utf-8'))
                    data["status"] = "ONLINE"
                    with lock:
                        telemetry_data = data
                except json.JSONDecodeError:
                    pass
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with lock:
                telemetry_data["status"] = f"BT LOST"
                bt_sock = None
            if 's' in locals() and s: s.close()
            time.sleep(2)

t = threading.Thread(target=bt_client_thread)
t.daemon = True
t.start()

def send_bt_command(cmd_str):
    """Sends a short command string to the Pi"""
    global bt_sock
    with lock:
        if bt_sock:
            try:
                # Type=CMD (4 bytes), Length=0. We encode command in the Type field for simplicity if length is 4
                # But safer to send as payload if length > 4.
                # Here we use the Type field for the Command itself if it's 4 chars (e.g. "TRK+")
                pkt = struct.pack("!4sI", cmd_str.encode('utf-8'), 0)
                bt_sock.sendall(pkt)
                print(f"[GS] Sent: {cmd_str}")
                return True
            except Exception as e:
                print(f"[GS] Send Error: {e}")
    return False

# --- WEB SERVER ---

@app.route('/')
def index():
    return render_template('dashboard.html', video_url="/proxy_video")

@app.route('/proxy_video')
def proxy_video():
    """Proxies MJPEG stream from Pi to Browser (Works over Tailscale)"""
    pi_stream_url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream"
    try:
        req = requests.get(pi_stream_url, stream=True, timeout=3)
        def generate():
            for chunk in req.iter_content(chunk_size=4096):
                yield chunk
        return Response(stream_with_context(generate()), 
                        content_type=req.headers['Content-Type'])
    except Exception:
        return "Video Signal Unreachable"

@app.route('/api/telemetry')
def api_telemetry():
    with lock:
        return jsonify(telemetry_data)

@app.route('/api/track', methods=['POST'])
def api_track():
    action = request.json.get('action')
    if action == 'start':
        send_bt_command("TRK+")
    elif action == 'stop':
        send_bt_command("TRK-")
    return jsonify({"status": "ok"})

@app.route('/api/capture', methods=['POST'])
def capture_image():
    """Downloads high-res image from Pi and saves UNIQUE file"""
    try:
        url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot"
        resp = requests.get(url, timeout=15)
        
        if resp.status_code == 200:
            timestamp = int(time.time())
            filename = f"snap_{timestamp}.jpg"
            filepath = os.path.join(CAPTURE_DIR, filename)
            
            with open(filepath, 'wb') as f:
                f.write(resp.content)
                
            return jsonify({"status": "success", "file": filename})
        else:
            return jsonify({"status": "error", "message": "Pi Error"}), 500
            
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/captures')
def list_captures():
    try:
        # Sort by newest first
        files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.endswith(".jpg")], reverse=True)
        return jsonify(files)
    except:
        return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
