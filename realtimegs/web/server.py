import threading
import os
import time
import json
from flask import Flask, Response, jsonify, render_template
from config import DATA_DIR

app = Flask(__name__)

# Global hardware references (Injected from main.py)
camera_ref = None
imu_ref = None

def run_server(camera_obj, imu_obj, port):
    global camera_ref, imu_ref
    camera_ref = camera_obj
    imu_ref = imu_obj
    
    # Run Flask in a daemon thread so it doesn't block the IMU loop
    t = threading.Thread(target=app.run, kwargs={
        'host': '0.0.0.0', 
        'port': port, 
        'threaded': True,
        'use_reloader': False
    })
    t.daemon = True
    t.start()

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if camera_ref:
                frame = camera_ref.get_streaming_frame()
                if frame:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry():
    if imu_ref:
        return jsonify(imu_ref.get_data())
    return jsonify({"error": "No IMU"})

@app.route('/capture')
def capture():
    """
    Saves image + JSON metadata for SfM/Splatting.
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    base_name = f"scan_{timestamp}"
    
    # 1. Get Synchronized IMU Data
    imu_data = imu_ref.get_data() if imu_ref else {}
    
    # 2. Save Image
    img_name = f"{base_name}.jpg"
    success = False
    if camera_ref:
        success = camera_ref.capture_high_res(os.path.join(DATA_DIR, img_name))
    
    # 3. Save Metadata (JSON)
    if success:
        json_name = f"{base_name}.json"
        metadata = {
            "image_path": img_name,
            "timestamp": timestamp,
            "pose_prior": {
                "quaternion_wxyz": imu_data.get('quaternion', [1,0,0,0]),
                "euler_deg": imu_data.get('orientation_euler', {}),
                "accel_ms2": imu_data.get('accel', {})
            }
        }
        
        with open(os.path.join(DATA_DIR, json_name), 'w') as f:
            json.dump(metadata, f, indent=4)
            
        return jsonify({"status": "captured", "file": img_name})
    
    return jsonify({"status": "error"})
