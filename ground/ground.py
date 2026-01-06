import socket
import threading
import struct
import json
import time
import os
import requests
from flask import Flask, render_template, jsonify, request

# --- CONFIGURATION ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  # <--- REPLACE WITH PI BLUETOOTH MAC
PI_WIFI_IP = "192.168.1.183"    # <--- REPLACE WITH PI WIFI IP
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
    "accel": {"x": 0, "y": 0, "z": 0},
    "orientation_euler": {"roll": 0, "pitch": 0, "yaw": 0}
}
lock = threading.Lock()

def bt_client_thread():
    """Background thread to fetch Telemetry over Bluetooth."""
    global telemetry_data
    
    while True:
        sock = None
        try:
            print(f"[GS] Connecting BT to {PI_BT_MAC}...")
            sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            sock.connect((PI_BT_MAC, 1))
            print("[GS] Bluetooth Connected!")
            
            with lock:
                telemetry_data["status"] = "ONLINE"

            while True:
                header = sock.recv(8)
                if not header: break
                type_bytes, length = struct.unpack("!4sI", header)
                
                payload = b''
                while len(payload) < length:
                    chunk = sock.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                if len(payload) != length: break

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
            if sock: sock.close()
            time.sleep(2)

t = threading.Thread(target=bt_client_thread)
t.daemon = True
t.start()

# --- WEB SERVER ---

@app.route('/')
def index():
    video_url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream"
    return render_template('dashboard.html', video_url=video_url)

@app.route('/api/telemetry')
def api_telemetry():
    with lock:
        return jsonify(telemetry_data)

@app.route('/api/capture', methods=['POST'])
def capture_image():
    """Downloads high-res image from Pi and saves to Laptop disk"""
    try:
        url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot"
        print(f"[GS] Requesting High Res Snap from {url}...")
        
        # Download from Pi (Timeout increased for 4K images)
        resp = requests.get(url, timeout=15)
        
        if resp.status_code == 200:
            timestamp = int(time.time())
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(CAPTURE_DIR, filename)
            
            with open(filepath, 'wb') as f:
                f.write(resp.content)
                
            print(f"[GS] Saved {len(resp.content)/1024:.1f}KB to {filename}")
            return jsonify({"status": "success", "file": filename, "size": len(resp.content)})
        else:
            return jsonify({"status": "error", "message": "Pi returned error"}), 500
            
    except Exception as e:
        print(f"[GS] Capture Failed: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
