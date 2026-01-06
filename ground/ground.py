import socket
import threading
import struct
import json
import time
from flask import Flask, render_template, jsonify

# --- CONFIGURATION ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  
PI_WIFI_IP = "192.168.1.183"  
PI_VIDEO_PORT = 8000

app = Flask(__name__)

# --- GLOBAL STATE ---
# Initial state avoids "Waiting..." hanging forever if connection is slow
telemetry_data = {
    "status": "CONNECTING...",
    "rssi": 0,
    "accel": {"x": 0, "y": 0, "z": 0},
    "orientation_euler": {"roll": 0, "pitch": 0, "yaw": 0}
}
lock = threading.Lock()

def bt_client_thread():
    """
    Background thread to fetch Telemetry over Bluetooth.
    Connects to the Pi's Bluetooth RFCOMM server.
    """
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
                # 1. Read Header (4 bytes type + 4 bytes len)
                header = sock.recv(8)
                if not header: break
                type_bytes, length = struct.unpack("!4sI", header)
                
                # 2. Read Payload
                payload = b''
                while len(payload) < length:
                    chunk = sock.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                if len(payload) != length:
                    print("[GS] Incomplete packet")
                    break

                # 3. Update State
                try:
                    data = json.loads(payload.decode('utf-8'))
                    # Inject status so dashboard knows we are live
                    data["status"] = "ONLINE"
                    with lock:
                        telemetry_data = data
                except json.JSONDecodeError:
                    print(f"[GS] Bad JSON: {payload}")
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with lock:
                telemetry_data["status"] = f"BT LOST"
            if sock: sock.close()
            time.sleep(2) # Retry delay

# Start Bluetooth Thread
t = threading.Thread(target=bt_client_thread)
t.daemon = True
t.start()

# --- WEB SERVER ---

@app.route('/')
def index():
    # Pass the Wi-Fi video URL to the template
    video_url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream"
    return render_template('dashboard.html', video_url=video_url)

@app.route('/api/telemetry')
def api_telemetry():
    with lock:
        return jsonify(telemetry_data)

if __name__ == "__main__":
    print("=========================================")
    print(f"   ARTEMIS HYBRID GS")
    print(f"   Dashboard: http://localhost:5000")
    print("=========================================")
    app.run(host='0.0.0.0', port=5000)
