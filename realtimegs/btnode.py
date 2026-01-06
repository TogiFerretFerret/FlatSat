import socket
import threading
import time
import json
import struct
import os
import sys
import io
import traceback
import subprocess # Added for RSSI commands

# --- HARDWARE IMPORTS ---
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

from config import DATA_DIR

class BluetoothNode:
    def __init__(self):
        self.server_sock = None
        self.client_sock = None
        self.running = True
        
        # CRITICAL: Lock to prevent Telemetry and Image threads 
        # from fighting over the socket and crashing connection.
        self.socket_lock = threading.Lock()
        
        # Motion Gating State
        self.last_img_size = 0
        self.last_img_time = 0
        self.FORCE_REFRESH_SEC = 5.0 # Send full frame every 5s anyway
        self.DIFF_THRESHOLD = 0.02   # 2% size change = Motion Detected
        
        # RSSI State
        self.cached_rssi = 0
        self.last_rssi_time = 0
        
        # Init Hardware
        self.camera = None
        if CAMERA_AVAILABLE:
            try:
                self.camera = CameraSystem()
                print("[System] Camera Initialized.")
            except Exception as e:
                print(f"[System] Camera Init Failed: {e}")

        self.imu = None
        if IMU:
            try:
                self.imu = IMU()
                print("[System] IMU Initialized.")
            except Exception as e:
                print(f"[System] IMU Init Failed: {e}")

    def start_server(self):
        # 1. Setup Bluetooth Config (Linux specific)
        print("[BT] Configuring Adapter...")
        os.system("sudo hciconfig hci0 piscan") # Make discoverable
        os.system("sdptool add SP")             # Advertise Serial Port

        try:
            self.server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.server_sock.bind(("00:00:00:00:00:00", 1)) # Bind to Any, Port 1
            self.server_sock.listen(1)
            
            print("[BT] Listening on RFCOMM Channel 1...")
            
            while self.running:
                print("[BT] Waiting for Ground Station...")
                self.client_sock, address = self.server_sock.accept()
                print(f"[BT] Connected to {address}")
                
                # CRITICAL: Blocking mode ensures we don't get Error 11 (Resource Unavailable)
                # when the buffer fills up. The OS will pause our script until space is available.
                self.client_sock.setblocking(True)
                
                # Pass the client MAC address to the handler for RSSI queries
                self.handle_client(address[0])
                
        except Exception as e:
            print(f"[BT] Server Error: {e}")
        finally:
            self.cleanup()

    def get_rssi(self, mac_address):
        """Queries the system for Signal Strength (RSSI)."""
        # Throttle: Only query every 2 seconds to prevent CPU spikes
        if time.time() - self.last_rssi_time < 2.0:
            return self.cached_rssi
            
        try:
            # Uses 'hcitool rssi <MAC>' command
            # Output format: "RSSI return value: -52"
            cmd = ["hcitool", "rssi", mac_address]
            result = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            result_str = result.decode('utf-8').strip()
            if "return value" in result_str:
                # Extract the number after the colon
                val = int(result_str.split(":")[1])
                self.cached_rssi = val
                self.last_rssi_time = time.time()
                return val
        except Exception:
            # hcitool might fail if connection momentarily drops or command doesn't exist
            pass
        return self.cached_rssi

    def send_packet(self, type_str, payload):
        """
        Thread-safe packet sender. 
        Splits large data (images) into chunks to prevent buffer overflow.
        """
        if not self.client_sock: return

        # Prepare Data
        try:
            if isinstance(payload, dict):
                data_bytes = json.dumps(payload).encode('utf-8')
            elif isinstance(payload, bytes):
                data_bytes = payload
            else:
                data_bytes = str(payload).encode('utf-8')
        except Exception as e:
            print(f"[BT] Encode Error: {e}")
            return

        total_len = len(data_bytes)
        
        # Construct Header [Type:4s][Len:I]
        header = struct.pack("!4sI", type_str.encode('utf-8'), total_len)

        # CRITICAL: The Lock!
        # Only one thread can write to the socket at a time.
        with self.socket_lock:
            try:
                # 1. Send Header
                self.client_sock.sendall(header)
                
                # 2. Send Payload (Chunked for stability)
                # Sending 75KB at once often chokes RFCOMM. 
                # We break it into 2048 byte chunks.
                CHUNK_SIZE = 2048
                for i in range(0, total_len, CHUNK_SIZE):
                    chunk = data_bytes[i:i+CHUNK_SIZE]
                    self.client_sock.sendall(chunk)
                    
            except Exception as e:
                print(f"[BT] Socket Error during send: {e}")
                try:
                    self.client_sock.close()
                except: pass
                self.client_sock = None

    def handle_client(self, client_mac):
        # Start listening for "SNAP" commands in background
        read_thread = threading.Thread(target=self.read_commands)
        read_thread.daemon = True
        read_thread.start()

        print("[BT] Streaming Telemetry...")
        while self.running and self.client_sock:
            # 1. Gather Telemetry
            if self.imu and self.imu.ready:
                telemetry = self.imu.get_data()
            else:
                telemetry = {
                    "status": "NO_SENSOR", 
                    "accel": {"x":0, "y":0, "z":0},
                    "timestamp": time.time()
                }
            
            # 2. Add RSSI Data
            telemetry["rssi"] = self.get_rssi(client_mac)
            
            # 3. Send (Protected by Lock inside send_packet)
            self.send_packet("TELE", telemetry)
            
            # Update Rate: 20Hz is nice, but if the image thread is busy,
            # this might just queue up.
            time.sleep(0.05) 

    def read_commands(self):
        """Reads incoming commands from the laptop."""
        while self.running and self.client_sock:
            try:
                # We assume the socket lock is NOT held during recv, 
                # which is standard. Recv blocks until data arrives.
                header = self.client_sock.recv(8)
                if not header: break
                
                cmd_type, cmd_len = struct.unpack("!4sI", header)
                
                cmd_payload = b''
                while len(cmd_payload) < cmd_len:
                    chunk = self.client_sock.recv(cmd_len - len(cmd_payload))
                    if not chunk: break
                    cmd_payload += chunk
                
                cmd = cmd_type.decode('utf-8')
                # print(f"[BT] Received: {cmd}") # Commented out to reduce spam

                if cmd == "SNAP":
                    self.send_image()

            except Exception as e:
                print(f"[BT] Read Loop Error: {e}")
                break
        print("[BT] Read Loop Ended.")

    def send_image(self):
        if not self.camera:
            self.send_packet("ERR_", "No Camera")
            return

        # print("[BT] Snap request...")
        stream = io.BytesIO()
        try:
            # FIX: Use set_controls for Quality instead of 'options' arg
            # 'JpegQuality': 25 gives decent preview speed. 
            # If you want Higher Quality for Splats, increase this number,
            # but your framerate will drop significantly.
            self.camera.picam2.set_controls({"JpegQuality": 25})
            
            # Capture to RAM
            self.camera.picam2.capture_file(stream, format="jpeg")
            
            stream.seek(0)
            img_bytes = stream.read()
            current_size = len(img_bytes)
            
            # 2. Motion Gating Logic (Minimizing Data)
            # Calculate percent difference in file size
            size_diff = 0
            if self.last_img_size > 0:
                size_diff = abs(current_size - self.last_img_size) / self.last_img_size
            
            time_since_last = time.time() - self.last_img_time

            # 3. Decision: Send or Skip?
            # Send if: Time elapsed > 5s OR Difference > 2%
            if (time_since_last > self.FORCE_REFRESH_SEC) or (size_diff > self.DIFF_THRESHOLD):
                # print(f"[BT] Sending Image ({current_size} bytes, diff {size_diff:.2%})...")
                self.send_packet("IMG_", img_bytes) 
                
                # Update State
                self.last_img_size = current_size
                self.last_img_time = time.time()
            else:
                # print(f"[BT] Skipped (diff {size_diff:.2%})")
                # Send a tiny packet saying "Nothing New" to satisfy the GS auto-loop
                self.send_packet("SKIP", b"")
            
        except Exception as e:
            # Added Traceback to debug "Image is Error" issues
            traceback.print_exc()
            print(f"[BT] Capture Error: {e}")
            self.send_packet("ERR_", f"Cam Fail: {str(e)}")

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
