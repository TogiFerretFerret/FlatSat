import socket
import threading
import time
import json
import struct
import os
import io
import sys
import subprocess
from flask import Flask, Response, send_file

# --- HARDWARE IMPORTS ---
try:
    from hardware.camera import CameraSystem
    from hardware.imu import IMU
except ImportError:
    print("Hardware libraries missing. Ensure hardware/ folder exists.")
    sys.exit(1)

# --- CONFIG ---
HTTP_PORT = 8000 # Video Stream Port

# Initialize Global Hardware
cam = CameraSystem()
imu = IMU()
cam_lock = threading.Lock() # Lock to prevent stream/snap collision

def configure_camera(width, height):
    """
    Safely reconfigures the Picamera2 instance for a specific resolution.
    """
    print(f"[System] Switching Camera to {width}x{height}...")
    try:
        # Stop if running
        try:
            cam.picam2.stop()
        except:
            pass # Already stopped

        # Create new config
        config = cam.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        cam.picam2.configure(config)
        cam.picam2.start()
        print(f"[System] Camera Active at {width}x{height}.")
    except Exception as e:
        print(f"[System] Camera Config Failed: {e}")

# --- STARTUP: Set to 1080p for Smooth Streaming ---
configure_camera(1920, 1080)

# Flask App for Video
app = Flask(__name__)

class HybridNode:
    def __init__(self):
        self.running = True
        
        # --- THREAD 1: BLUETOOTH TELEMETRY ---
        # Runs in background so video doesn't block it
        self.bt_thread = threading.Thread(target=self.run_bt_server)
        self.bt_thread.daemon = True
        self.bt_thread.start()

    def get_rssi(self, mac):
        """Get Signal Strength from OS"""
        try:
            cmd = ["hcitool", "rssi", mac]
            res = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            return int(res.decode().split(":")[1])
        except:
            return 0

    def get_cpu_temp(self):
        """Get Raspberry Pi CPU Temperature"""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return float(f.read()) / 1000.0
        except:
            return 0.0

    def run_bt_server(self):
        """Dedicated Thread for Realtime Telemetry over Bluetooth"""
        print("[BT] Configuring Adapter...")
        os.system("sudo hciconfig hci0 piscan") # Make discoverable
        os.system("sdptool add SP")             # Advertise Serial Port

        try:
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
                        
                        # 2. Add Extra System Stats
                        data["rssi"] = self.get_rssi(addr[0])
                        data["cpu_temp"] = self.get_cpu_temp()
                        
                        # 3. Serialize
                        msg = json.dumps(data).encode('utf-8')
                        
                        # 4. Send (Header + Body)
                        header = struct.pack("!4sI", b"TELE", len(msg))
                        client.sendall(header + msg)
                        
                        # 20Hz Update Rate
                        time.sleep(0.05)
                        
                except Exception as e:
                    print(f"[BT] Disconnected: {e}")
                    if client: 
                        try: client.close()
                        except: pass
                    time.sleep(1)
        except Exception as e:
            print(f"[BT] Critical Server Error: {e}")

# --- FLASK VIDEO STREAMING (WIFI) ---

@app.route('/stream')
def stream():
    """
    High Bandwidth Video Stream over Wi-Fi (Runs at 1080p).
    """
    def generate():
        stream = io.BytesIO()
        while True:
            with cam_lock:
                # Capture frame (uses current 1080p config)
                cam.picam2.capture_file(stream, format="jpeg")
            
            # Read bytes
            stream.seek(0)
            frame = stream.read()
            
            # Reset buffer
            stream.seek(0)
            stream.truncate()
            
            # Yield MJPEG chunk
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # Limit to ~30 FPS
            time.sleep(0.03)
            
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    """
    Captures a SINGLE High-Res (4.6K) image.
    1. Lock Camera (Pauses stream).
    2. Switch to 4.6K.
    3. Capture.
    4. Switch back to 1080p.
    5. Unlock (Resumes stream).
    """
    stream = io.BytesIO()
    
    with cam_lock:
        print("[Snap] Switching to 4.6K...")
        configure_camera(4608, 2592)
        
        # Optional: Sleep briefly to let auto-exposure settle on the new resolution
        # time.sleep(0.5) 
        
        print("[Snap] Capturing...")
        cam.picam2.capture_file(stream, format="jpeg")
        
        print("[Snap] Reverting to 1080p...")
        configure_camera(1920, 1080)
    
    stream.seek(0)
    return send_file(stream, mimetype='image/jpeg', download_name='snap.jpg')

if __name__ == "__main__":
    node = HybridNode()
    
    print("==============================================")
    print(f"   HYBRID NODE ACTIVE")
    print(f"   1. Bluetooth: Telemetry & Command Plane")
    print(f"   2. Wi-Fi: Video Plane (1080p Stream / 4.6K Snap)")
    print(f"   http://<PI_IP>:{HTTP_PORT}/stream")
    print("==============================================")
    
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
