import socket
import threading
import struct
import json
import time
import io
from flask import Flask, render_template_string, send_file, request, jsonify, Response

# --- CONFIGURATION ---
# REPLACE with your Pi's MAC Address
PI_MAC_ADDRESS = "D8:3A:DD:3C:12:16" 

app = Flask(__name__)

class SingleNodeGS:
    def __init__(self, mac):
        self.mac = mac
        self.sock = None
        
        # Data storage (RAM ONLY)
        self.telemetry = {"status": "STARTING..."}
        self.last_image_bytes = None 
        self.last_image_ts = 0
        
        self.connected = False
        self.streaming = False
        self.lock = threading.Lock()
        
        # Start connection manager in background
        self.thread = threading.Thread(target=self.connect_loop)
        self.thread.daemon = True
        self.thread.start()

    def connect_loop(self):
        """Persistent connection manager that auto-reconnects."""
        while True:
            try:
                print(f"[GS] Connecting to {self.mac}...")
                self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.sock.connect((self.mac, 1))
                
                self.connected = True
                print("[GS] Connected!")
                
                # Start listening
                self.receive_loop()
                
            except Exception as e:
                print(f"[GS] Connection Failed: {e}")
                self.connected = False
                with self.lock:
                    self.telemetry = {"status": f"OFFLINE ({str(e)})"}
                # Wait before retrying
                time.sleep(5)

    def send_command(self, cmd_type, payload=""):
        if not self.connected or not self.sock: return
        try:
            data = payload.encode('utf-8')
            header = struct.pack("!4sI", cmd_type.encode('utf-8'), len(data))
            self.sock.sendall(header + data)
        except Exception as e:
            print(f"[GS] Send Error: {e}")
            self.connected = False

    def receive_loop(self):
        """Main data pump. Blocks until socket dies."""
        while self.connected:
            try:
                # 1. Read Header (8 bytes)
                header = self.sock.recv(8)
                if not header: break
                
                type_bytes, length = struct.unpack("!4sI", header)
                pkt_type = type_bytes.decode('utf-8')

                # 2. Read Payload (Handle fragmentation)
                payload = b''
                while len(payload) < length:
                    chunk = self.sock.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk
                
                if len(payload) != length: break

                # 3. Process Data
                if pkt_type == "TELE":
                    with self.lock:
                        self.telemetry = json.loads(payload.decode('utf-8'))
                
                elif pkt_type == "IMG_":
                    print(f"[GS] Image in RAM ({len(payload)} bytes)")
                    with self.lock:
                        self.last_image_bytes = payload
                        self.last_image_ts = time.time()
                    
                    # Auto-loop if streaming
                    if self.streaming:
                        time.sleep(0.05)
                        self.send_command("SNAP")
                
                elif pkt_type == "SKIP":
                    # Pi says "Image hasn't changed", so we request again immediately
                    if self.streaming:
                        time.sleep(0.05)
                        self.send_command("SNAP")

            except Exception as e:
                print(f"[GS] Receive Error: {e}")
                break
        
        # Cleanup if loop exits
        if self.sock: 
            try: self.sock.close()
            except: pass
        self.connected = False

# Init the Global Node
node = SingleNodeGS(PI_MAC_ADDRESS)

# --- FLASK ROUTES ---

@app.route('/')
def index():
    return render_template_string("""
    <html>
    <head>
        <title>Artemis Ground Station</title>
        <style>
            body { font-family: monospace; background: #111; color: #0f0; padding: 20px; }
            .box { border: 1px solid #444; padding: 15px; margin-bottom: 20px; background: #000; }
            .img-box { text-align: center; min-height: 480px; display: flex; align-items: center; justify-content: center; background: #050505; }
            img { max-width: 100%; border: 1px solid #333; }
            button { padding: 15px 30px; font-size: 1.2em; font-weight: bold; cursor: pointer; border: none; margin-right: 10px; }
            .btn-snap { background: #008800; color: white; }
            .btn-stream { background: #004400; color: #aaa; }
            .streaming { background: #ff0000; color: white; }
            pre { font-size: 1.1em; }
        </style>
        <script>
            // Poll for data updates
            setInterval(async () => {
                try {
                    let res = await fetch('/api/data');
                    let data = await res.json();
                    
                    // Update Telemetry Text
                    document.getElementById('telemetry').innerText = JSON.stringify(data.telemetry, null, 2);
                    
                    // Update Image if Timestamp changed
                    let img = document.getElementById('live-img');
                    if(data.image_ts != img.dataset.ts) {
                        img.src = "/img_ram?t=" + data.image_ts;
                        img.dataset.ts = data.image_ts;
                    }
                } catch(e) {}
            }, 100);

            async function toggleStream() {
                let btn = document.getElementById('btn-stream');
                let res = await fetch('/toggle_stream', {method: 'POST'});
                let status = await res.json();
                
                if(status.streaming) {
                    btn.innerText = "STOP STREAM";
                    btn.classList.add("streaming");
                } else {
                    btn.innerText = "START AUTO-STREAM";
                    btn.classList.remove("streaming");
                }
            }
            
            async function snap() {
                await fetch('/snap', {method: 'POST'});
            }
        </script>
    </head>
    <body>
        <h1>ARTEMIS ONE // GROUND STATION</h1>
        
        <div class="box img-box">
            <img id="live-img" data-ts="0" src="" alt="WAITING FOR DATA...">
        </div>

        <div class="box">
            <button class="btn-snap" onclick="snap()">SINGLE SNAP</button>
            <button id="btn-stream" class="btn-stream" onclick="toggleStream()">START AUTO-STREAM</button>
        </div>

        <div class="box">
            <h3>AVIONICS TELEMETRY</h3>
            <pre id="telemetry">Connecting...</pre>
        </div>
    </body>
    </html>
    """)

@app.route('/api/data')
def api_data():
    """JSON API for JS Polling"""
    with node.lock:
        return jsonify({
            "telemetry": node.telemetry,
            "image_ts": node.last_image_ts
        })

@app.route('/img_ram')
def img_ram():
    """Serves the image directly from RAM (BytesIO)"""
    img_data = None
    with node.lock:
        if node.last_image_bytes:
            img_data = io.BytesIO(node.last_image_bytes)
    
    if img_data:
        img_data.seek(0)
        return send_file(img_data, mimetype='image/jpeg')
    return "", 404

@app.route('/snap', methods=['POST'])
def snap():
    node.send_command("SNAP")
    return jsonify({"status": "ok"})

@app.route('/toggle_stream', methods=['POST'])
def toggle_stream():
    node.streaming = not node.streaming
    if node.streaming:
        node.send_command("SNAP") # Kickstart
    return jsonify({"streaming": node.streaming})

if __name__ == "__main__":
    # threaded=True is essential for Flask to handle the background connection
    app.run(host='0.0.0.0', port=5000, threaded=True)
