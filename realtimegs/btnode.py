import socket
import threading
import time
import json
import struct
import os
import sys
import io  # Added for RAM-based image capture (Faster)

# --- HARDWARE IMPORTS (With Safety Checks) ---
try:
    from hardware.camera import CameraSystem
    CAMERA_AVAILABLE = True
except ImportError:
    print("[System] Camera module not found.")
    CAMERA_AVAILABLE = False
except Exception as e:
    print(f"[System] Camera Error: {e}")
    CAMERA_AVAILABLE = False

try:
    from hardware.imu import IMU
except ImportError:
    print("[System] IMU module/libraries not found. Running in Sensorless Mode.")
    IMU = None

# Config
from config import DATA_DIR

class BluetoothNode:
    def __init__(self):
        self.server_sock = None
        self.client_sock = None
        self.running = True
        
        # --- INIT CAMERA ---
        if CAMERA_AVAILABLE:
            try:
                self.camera = CameraSystem()
                print("[System] Camera Initialized.")
            except Exception as e:
                print(f"[System] Camera Init Failed: {e}")
                self.camera = None
        else:
            self.camera = None

        # --- INIT IMU ---
        self.imu = None
        if IMU:
            try:
                self.imu = IMU()
                print("[System] IMU Initialized.")
            except Exception as e:
                print(f"[System] IMU Init Failed: {e}")

    def start_server(self):
        """Sets up the RFCOMM Bluetooth Server."""
        try:
            self.server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            
            # FIXED: Use explicit BDADDR_ANY instead of empty string
            # This fixes "bad bluetooth address" on newer Linux kernels
            self.server_sock.bind(("00:00:00:00:00:00", 1)) 
            
            self.server_sock.listen(1)
            
            print("[BT] Listening on RFCOMM Channel 1...")
            
            while self.running:
                print("[BT] Waiting for Ground Station...")
                self.client_sock, address = self.server_sock.accept()
                print(f"[BT] Connected to {address}")
                # Set socket to blocking mode for reliability
                self.client_sock.setblocking(True)
                self.handle_client()
                
        except Exception as e:
            print(f"[BT] Server Error: {e}")
        finally:
            self.cleanup()

    def send_packet(self, type_str, payload):
        if not self.client_sock: return
        try:
            if isinstance(payload, dict):
                data_bytes = json.dumps(payload).encode('utf-8')
            elif isinstance(payload, bytes):
                data_bytes = payload
            else:
                data_bytes = str(payload).encode('utf-8')

            # Header: [Type:4s][Len:I] (8 bytes total)
            header = struct.pack("!4sI", type_str.encode('utf-8'), len(data_bytes))
            self.client_sock.sendall(header + data_bytes)
        except Exception as e:
            print(f"[BT] Send Error: {e}")
            # If send fails, assume client disconnected
            if self.client_sock:
                try:
                    self.client_sock.close()
                except: pass
            self.client_sock = None

    def handle_client(self):
        """Main loop for a connected client."""
        # Start command listener
        read_thread = threading.Thread(target=self.read_commands)
        read_thread.daemon = True
        read_thread.start()

        print("[BT] Streaming Realtime Telemetry...")
        while self.running and self.client_sock:
            # 1. Get Data (or Mock it if no IMU)
            if self.imu and self.imu.ready:
                telemetry = self.imu.get_data()
            else:
                # SENSORLESS FALLBACK
                telemetry = {
                    "status": "NO_SENSOR", 
                    "accel": {"x":0, "y":0, "z":0},
                    "temp": 0,
                    "timestamp": time.time()
                }
            
            # 2. Send Packet
            self.send_packet("TELE", telemetry)
            
            # FAST UPDATES: 20Hz (0.05s). 
            # Bluetooth RFCOMM can handle this easily for small text packets.
            time.sleep(0.05) 

    def read_commands(self):
        while self.running and self.client_sock:
            try:
                # Blocking read for the header
                header = self.client_sock.recv(8)
                if not header: break
                
                cmd_type, cmd_len = struct.unpack("!4sI", header)
                
                # Blocking read for payload
                cmd_payload = b''
                while len(cmd_payload) < cmd_len:
                    chunk = self.client_sock.recv(cmd_len - len(cmd_payload))
                    if not chunk: break
                    cmd_payload += chunk
                
                cmd = cmd_type.decode('utf-8')
                print(f"[BT] CMD Received: {cmd}")

                if cmd == "SNAP":
                    self.send_image()

            except Exception as e:
                print(f"[BT] Read Error: {e}")
                break
        print("[BT] Client Disconnected.")

    def send_image(self):
        if not self.camera:
            self.send_packet("ERR_", "No Camera Hardware")
            return

        print("[BT] Capturing to RAM...")
        # OPTIMIZATION: Use BytesIO to capture to RAM instead of SD Card
        # This is much faster for "Realtime" feel
        stream = io.BytesIO()
        
        try:
            # Picamera2 native capture to stream
            self.camera.picam2.capture_file(stream, format="jpeg")
            
            # Get the bytes
            stream.seek(0)
            img_bytes = stream.read()
            
            print(f"[BT] Sending {len(img_bytes)} bytes...")
            self.send_packet("IMG_", img_bytes)
            print("[BT] Transfer Complete.")
            
        except Exception as e:
            print(f"[BT] Snap Error: {e}")
            self.send_packet("ERR_", "Capture Failed")

    def cleanup(self):
        self.running = False
        if self.client_sock: 
            try: self.client_sock.close()
            except: pass
        if self.server_sock: 
            try: self.server_sock.close()
            except: pass
        if self.camera: self.camera.stop()

if __name__ == "__main__":
    node = BluetoothNode()
    try:
        node.start_server()
    except KeyboardInterrupt:
        node.cleanup()
