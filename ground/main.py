import socket
import threading
import struct
import json
import time
import os
import requests
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, send_from_directory

PI_BT_MAC = "D8:3A:DD:3C:12:16" 
PI_WIFI_IP = "192.168.1.183"    
PI_VIDEO_PORT = 8000
CAPTURE_DIR = os.path.join(os.getcwd(), "captures")
if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

app = Flask(__name__)
telemetry_data = {"status": "CONNECTING...", "heading": 0}
bt_sock = None
lock = threading.Lock()

def bt_client_thread():
    global bt_sock, telemetry_data
    while True:
        try:
            print(f"[GS] Connecting BT...")
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.connect((PI_BT_MAC, 1))
            with lock: 
                bt_sock = s
                telemetry_data["status"] = "ONLINE"
            
            while True:
                header = s.recv(8)
                if not header: break
                _, length = struct.unpack("!4sI", header)
                payload = b''
                while len(payload) < length:
                    chunk = s.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                try:
                    data = json.loads(payload.decode('utf-8'))
                    data["status"] = "ONLINE"
                    with lock: telemetry_data.update(data)
                except: pass
        except:
            with lock: telemetry_data["status"] = "BT LOST"
            if bt_sock: bt_sock.close()
            time.sleep(2)

threading.Thread(target=bt_client_thread, daemon=True).start()

@app.route('/')
def index(): return render_template('dashboard.html', video_url="/proxy_video")

@app.route('/proxy_video')
def proxy_video():
    try:
        req = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream", stream=True, timeout=2)
        return Response(stream_with_context(req.iter_content(chunk_size=4096)), content_type=req.headers['Content-Type'])
    except: return "No Signal"

@app.route('/api/telemetry')
def api_telemetry():
    with lock: return jsonify(telemetry_data)

@app.route('/api/capture', methods=['POST'])
def capture_image():
    try:
        resp = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot", timeout=15)
        if resp.status_code == 200:
            fn = f"snap_{int(time.time())}.jpg"
            with open(os.path.join(CAPTURE_DIR, fn), 'wb') as f: f.write(resp.content)
            return jsonify({"status": "success", "file": fn})
    except: pass
    return jsonify({"status": "error"})

@app.route('/api/download_csv')
def download_csv():
    """Proxy the CSV file from Pi to Browser"""
    try:
        url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/download_log"
        resp = requests.get(url, timeout=5)
        return Response(
            resp.content,
            mimetype="text/csv",
            headers={"Content-disposition": "attachment; filename=mission_data.csv"}
        )
    except: return "Log Download Failed", 500

@app.route('/api/captures')
def list_captures():
    try: return jsonify(sorted([f for f in os.listdir(CAPTURE_DIR) if f.endswith(".jpg")], reverse=True))
    except: return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename): return send_from_directory(CAPTURE_DIR, filename)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
