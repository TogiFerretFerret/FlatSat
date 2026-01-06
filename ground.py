import socket
import threading
import struct
import json
import time
from flask import Flask, render_template_string, jsonify

# --- CONFIGURATION ---
# 1. Run 'hcitool dev' on Pi to get this
PI_BT_MAC = "D8:3A:DD:3C:12:16"  

# 2. Run 'hostname -I' on Pi to get this
PI_WIFI_IP = "192.168.1.183"    
PI_VIDEO_PORT = 8000

app = Flask(__name__)

# --- GLOBAL STATE ---
telemetry_data = {"status": "Waiting for BT..."}
lock = threading.Lock()

def bt_client_thread():
    """
    Background thread to fetch Telemetry over Bluetooth.
    This keeps the control loop fast and separate from video.
    """
    global telemetry_data
    
    while True:
        try:
            print(f"[GS] Connecting BT to {PI_BT_MAC}...")
            sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            sock.connect((PI_BT_MAC, 1))
            print("[GS] Bluetooth Connected!")
            
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
                
                # 3. Update State
                data = json.loads(payload.decode('utf-8'))
                with lock:
                    telemetry_data = data
                    
        except Exception as e:
            print(f"[GS] BT Error: {e}")
            with lock:
                telemetry_data = {"status": f"BT DISCONNECTED ({str(e)})"}
            time.sleep(5) # Retry delay

# Start BT Thread
t = threading.Thread(target=bt_client_thread)
t.daemon = True
t.start()

# --- WEB DASHBOARD ---

@app.route('/')
def index():
    # The trick: We tell the BROWSER to fetch the video directly from the Pi over Wi-Fi.
    # The Python script doesn't proxy the video, saving CPU.
    video_url = f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream"
    
    return render_template_string("""
    <html>
    <head>
        <title>Artemis Hybrid GS</title>
        <style>
            body { background: #111; color: #0f0; font-family: 'Courier New', monospace; padding: 20px; }
            .container { display: grid; grid-template-columns: 2fr 1fr; gap: 20px; }
            .box { background: #000; border: 1px solid #444; padding: 20px; }
            img { width: 100%; border: 1px solid #0f0; }
            h1 { border-bottom: 2px solid #0f0; padding-bottom: 10px; }
            .stat { font-size: 1.5em; margin-bottom: 15px; }
            .label { color: #888; font-size: 0.8em; display: block; margin-bottom: 5px; }
            .value { color: #fff; font-weight: bold; }
        </style>
        <script>
            // Poll for Bluetooth Telemetry updates
            setInterval(async () => {
                let res = await fetch('/api/telemetry');
                let data = await res.json();
                
                document.getElementById('tel-raw').innerText = JSON.stringify(data, null, 2);
                
                // Update Gauges if data is valid
                if(data.accel) {
                    document.getElementById('rssi').innerText = data.rssi || "N/A";
                    document.getElementById('accel-x').innerText = data.accel.x.toFixed(2);
                    document.getElementById('accel-y').innerText = data.accel.y.toFixed(2);
                    document.getElementById('accel-z').innerText = data.accel.z.toFixed(2);
                }
            }, 100);
        </script>
    </head>
    <body>
        <h1>ARTEMIS HYBRID LINK</h1>
        <div class="container">
            <!-- WIFI DATA PLANE (VIDEO) -->
            <div class="box">
                <h2>OPTICAL FEED (WIFI)</h2>
                <img src="{{ video_url }}" onerror="this.src=''; this.alt='VIDEO DISCONNECTED (Check Wi-Fi)'">
            </div>

            <!-- BLUETOOTH CONTROL PLANE (TELEMETRY) -->
            <div class="box">
                <h2>AVIONICS (BLUETOOTH)</h2>
                
                <div class="stat">
                    <span class="label">SIGNAL STRENGTH (RSSI)</span>
                    <span id="rssi" class="value">--</span> <span style="font-size:0.6em">dBm</span>
                </div>
                
                <div class="stat">
                    <span class="label">ACCELERATION (m/s²)</span>
                    X: <span id="accel-x" class="value">0.00</span><br>
                    Y: <span id="accel-y" class="value">0.00</span><br>
                    Z: <span id="accel-z" class="value">0.00</span>
                </div>

                <hr style="border-color:#333">
                <span class="label">RAW PACKET:</span>
                <pre id="tel-raw" style="color:#aaa; font-size:0.7em;">Waiting...</pre>
            </div>
        </div>
    </body>
    </html>
    """, video_url=video_url)

@app.route('/api/telemetry')
def api_telemetry():
    with lock:
        return jsonify(telemetry_data)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
