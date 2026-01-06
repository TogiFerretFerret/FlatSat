import socket
import threading
import struct
import json
import time
import os
from flask import Flask, render_template_string, send_file, request

# --- CONFIGURATION ---
# Add all your nodes here!
NODES = {
    "NODE_1 (IMU+CAM)": "D8:3A:DD:3C:12:16", # <-- Replace with Pi 1 MAC
}

SAVE_DIR = "ground_data"
if not os.path.exists(SAVE_DIR): os.makedirs(SAVE_DIR)

app = Flask(__name__)

# --- CONNECTION MANAGER ---
class NodeConnection:
    def __init__(self, name, mac):
        self.name = name
        self.mac = mac
        self.sock = None
        self.telemetry = {"status": "CONNECTING..."}
        self.last_image = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Start connection thread automatically
        t = threading.Thread(target=self.connect_loop)
        t.daemon = True
        t.start()

    def connect_loop(self):
        while True:
            try:
                print(f"[{self.name}] Connecting to {self.mac}...")
                self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.sock.connect((self.mac, 1))
                self.connected = True
                print(f"[{self.name}] Connected!")
                
                self.receive_loop()
                
            except Exception as e:
                self.connected = False
                self.telemetry = {"status": f"OFFLINE ({e})"}
                time.sleep(5) # Retry delay

    def send_command(self, cmd_type, payload=""):
        if not self.connected: return
        try:
            data = payload.encode('utf-8')
            header = struct.pack("!4sI", cmd_type.encode('utf-8'), len(data))
            self.sock.sendall(header + data)
        except:
            self.connected = False

    def receive_loop(self):
        while self.connected:
            try:
                # Read Header
                header = self.sock.recv(8)
                if not header: break
                
                type_bytes, length = struct.unpack("!4sI", header)
                pkt_type = type_bytes.decode('utf-8')

                # Read Payload
                payload = b''
                while len(payload) < length:
                    chunk = self.sock.recv(length - len(payload))
                    if not chunk: break
                    payload += chunk

                # Handle Data
                if pkt_type == "TELE":
                    with self.lock:
                        self.telemetry = json.loads(payload.decode('utf-8'))
                
                elif pkt_type == "IMG_":
                    print(f"[{self.name}] Received Image!")
                    fname = f"{self.name.replace(' ', '_')}_{int(time.time())}.jpg"
                    fpath = os.path.join(SAVE_DIR, fname)
                    with open(fpath, "wb") as f:
                        f.write(payload)
                    self.last_image = fpath

            except Exception as e:
                print(f"[{self.name}] Error: {e}")
                break
        
        self.sock.close()
        self.connected = False

# Initialize Connections
connections = {}
for name, mac in NODES.items():
    connections[name] = NodeConnection(name, mac)

# --- WEB DASHBOARD ---
@app.route('/')
def index():
    # Gather data for template
    nodes_data = []
    for name, conn in connections.items():
        with conn.lock:
            nodes_data.append({
                "name": name,
                "connected": conn.connected,
                "telemetry": json.dumps(conn.telemetry, indent=2),
                "image": conn.last_image
            })
            
    return render_template_string("""
    <html>
    <head>
        <title>Swarm Control</title>
        <meta http-equiv="refresh" content="2">
        <style>
            body { font-family: monospace; background: #222; color: #ddd; padding: 20px; }
            .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(400px, 1fr)); gap: 20px; }
            .node { background: #000; border: 1px solid #444; padding: 15px; }
            .online { border-color: #0f0; }
            .offline { border-color: #f00; }
            img { width: 100%; border: 1px solid #555; margin-top: 10px; }
            button { background: #008800; color: white; padding: 10px; width: 100%; border: none; cursor: pointer; }
        </style>
    </head>
    <body>
        <h1>ARTEMIS SWARM DASHBOARD</h1>
        <div class="grid">
            {% for node in nodes %}
            <div class="node {{ 'online' if node.connected else 'offline' }}">
                <h2>{{ node.name }}</h2>
                <pre>{{ node.telemetry }}</pre>
                
                {% if node.image %}
                    <img src="/img_view?path={{ node.image }}">
                {% else %}
                    <div style="padding:20px; text-align:center; color:#555;">NO IMAGE</div>
                {% endif %}
                
                <br><br>
                <form action="/snap" method="post">
                    <input type="hidden" name="node_name" value="{{ node.name }}">
                    <button type="submit">REQUEST PHOTO</button>
                </form>
            </div>
            {% endfor %}
        </div>
    </body>
    </html>
    """, nodes=nodes_data)

@app.route('/img_view')
def img_view():
    path = request.args.get('path')
    if path and os.path.exists(path):
        return send_file(path)
    return "Error"

@app.route('/snap', methods=['POST'])
def snap():
    name = request.form.get('node_name')
    if name in connections:
        connections[name].send_command("SNAP")
    return f"Snap requested for {name}. <a href='/'>Back</a>"

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
