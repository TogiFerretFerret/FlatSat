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
CAPTURE_DIR = os.path.join(os.getcwd(), "captures")

if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

app = Flask(__name__)

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
                # 1. Read Header (4 bytes "TELE" + 4 bytes Length)
                header = s.recv(8)
                if not header: break
                
                type_bytes, length = struct.unpack("!4sI", header)
                
                # 2. Read Payload
                payload = b''
                while len(payload) < length:
                    chunk = s.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                # 3. Parse JSON
                try:
                    new_data = json.loads(payload.decode('utf-8'))
                    new_data["status"] = "ONLINE"
                    
                    with data_lock: 
                        telemetry_data = new_data
                except json.JSONDecodeError:
                    print("[GS] Corrupt Packet")
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with data_lock: telemetry_data["status"] = "BT LOST"
            if bt_sock: 
                try: bt_sock.close()
                except: pass
                bt_sock = None
            time.sleep(2)

# Start BT in background
threading.Thread(target=bt_client_thread, daemon=True).start()

# --- FLASK ROUTES ---

@app.route('/')
def index():
    # Serve the dashboard
    return render_template('dashboard.html')

@app.route('/telemetry_stream')
def telemetry_stream():
    """
    The Magic Bridge:
    Takes the latest data from the Bluetooth thread and pushes it 
    to the Browser via Server-Sent Events (SSE).
    """
    def event_stream():
        while True:
            with data_lock:
                json_data = json.dumps(telemetry_data)
            
            # SSE Format: "data: {json}\n\n"
            yield f"data: {json_data}\n\n"
            
            # Push at 4Hz (250ms) to match Dashboard refresh rate
            time.sleep(0.25)

    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/stream')
def proxy_video():
    """ Proxy the MJPEG stream from the Pi over WiFi """
    try:
        req = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream", stream=True, timeout=2)
        return Response(stream_with_context(req.iter_content(chunk_size=4096)), content_type=req.headers['Content-Type'])
    except:
        # Return a placeholder or 404 if video is down
        return "Video Link Failed", 404

@app.route('/snapshot', methods=['POST'])
def proxy_snapshot():
    """ Proxy the High-Res Snapshot request to the Pi over WiFi """
    try:
        # Trigger snapshot on Pi
        resp = requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot", timeout=15)
        
        if resp.status_code == 200:
            # Save locally on Ground Station
            fn = f"snap_{int(time.time())}.jpg"
            full_path = os.path.join(CAPTURE_DIR, fn)
            with open(full_path, 'wb') as f:
                f.write(resp.content)
            return jsonify({"status": "success", "file": fn})
            
    except Exception as e:
        print(f"[GS] Snapshot Failed: {e}")
        
    return jsonify({"status": "error"}), 500

@app.route('/api/captures')
def list_captures():
    """ List local files for the gallery """
    try:
        files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.endswith(".jpg")], reverse=True)
        return jsonify(files)
    except: return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

if __name__ == "__main__":
    # Run on port 5000 (standard Flask port)
    app.run(host='0.0.0.0', port=5000, threaded=True)
