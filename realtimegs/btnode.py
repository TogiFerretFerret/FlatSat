import socket
import threading
import time
import json
import struct
import os
import io
import subprocess
from flask import Flask, Response

# --- HARDWARE ---
# We reuse your existing hardware drivers
try:
    from hardware.camera import CameraSystem
    from hardware.imu import IMU
except ImportError:
    print("Hardware libraries missing. Ensure hardware/ folder exists.")
    sys.exit(1)

# --- CONFIG ---
HTTP_PORT = 8000 # Video Stream Port

# Global Hardware
cam = CameraSystem()
imu = IMU()

# Flask App for Video
app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        self.bt_sock = None
        self.client_sock = None
        
        # --- THREAD 1: BLUETOOTH TELEMETRY ---
        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

        # --- THREAD 2: HTTP VIDEO STREAM ---
        # Flask runs on the main thread usually, or we can thread it.
        # We'll run Flask at the end of main().

    def get_rssi(self, mac):
        try:
            cmd = ["hcitool", "rssi", mac]
            res = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            return int(res.decode().split(":")[1])
        except:
            return 0

    def run_bt_server(self):
        """Dedicated Thread for Realtime Telemetry over Bluetooth"""
        print("[BT] Configuring Adapter...")
        os.system("sudo hciconfig hci0 piscan")
        os.system("sdptool add SP")

        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server_sock.bind(("00:00:00:00:00:00", 1))
        server_sock.listen(1)

        print("[BT] Ready for Telemetry Connection...")

        while self.running:
            try:
                client, addr = server_sock.accept()
                print(f"[BT] Ground Station Connected: {addr}")
                
                while self.running:
                    # 1. Get Sensor Data
                    data = imu.get_data() if imu else {"status": "no_imu"}
                    data["rssi"] = self.get_rssi(addr[0])
                    
                    # 2. Serialize
                    msg = json.dumps(data).encode('utf-8')
                    
                    # 3. Send (Header + Body)
                    # We send this blindly. If it fails, we assume disconnect.
                    header = struct.pack("!4sI", b"TELE", len(msg))
                    client.sendall(header + msg)
                    
                    # 20Hz Update Rate (Super smooth telemetry)
                    time.sleep(0.05)
                    
            except Exception as e:
                print(f"[BT] Disconnected: {e}")
                if client: client.close()
                time.sleep(1)

# --- FLASK VIDEO STREAMING ---

@app.route('/stream')
def stream():
    """
    High Bandwidth Video Stream over Wi-Fi.
    Allows for 1080p or higher without choking Bluetooth.
    """
    def generate():
        stream = io.BytesIO()
        while True:
            # Capture to RAM
            # quality=50 is a good balance for Wi-Fi (looks great, low lag)
            # You can set this to quality=90 for splatting datasets
            cam.picam2.capture_file(stream, format="jpeg", options={"quality": 50})
            stream.seek(0)
            frame = stream.read()
            stream.seek(0)
            stream.truncate()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # Limit to 30 FPS to save CPU
            time.sleep(0.03)
            
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    node = HybridNode()
    
    print("==============================================")
    print(f"   HYBRID NODE ACTIVE")
    print(f"   1. Connect Bluetooth for Telemetry")
    print(f"   2. Video Stream at http://<PI_IP>:{HTTP_PORT}/stream")
    print("==============================================")
    
    # Run Flask (Blocks Main Thread)
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
