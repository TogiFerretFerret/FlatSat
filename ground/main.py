import socket
import threading
import struct
import json
import time
import os
import requests
import sys
import glob
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, send_from_directory
from werkzeug.utils import secure_filename
from lunar_landing_analyzer import analyze_image
from stitching import stitch_images

# --- CONFIG ---
PI_BT_MAC = "D8:3A:DD:3C:12:16"  # CHANGE THIS to your Pi's MAC
PI_WIFI_IP = "cubesat.local"     # Use mDNS hostname directly
PI_VIDEO_PORT = 8000

# Robust Path Handling
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
if not os.path.exists(CAPTURE_DIR): os.makedirs(CAPTURE_DIR)

app = Flask(__name__, template_folder="web/templates", static_folder="web/static")

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Cache-Control', 'no-cache')
    response.headers.add('X-Accel-Buffering', 'no')
    return response

# Global State
telemetry_data = {"status": "CONNECTING...", "sys": {}, "cam_meta": {}}
data_lock = threading.Lock()

# --- MAPPING STATE ---
class MappingSession:
    def __init__(self):
        self.active = False
        self.frames = []
        self.master_map_fn = None
        self.analysis_fn = None
        self.thread = None
        self.lock = threading.Lock()

    def start(self):
        with self.lock:
            if self.active: return
            self.active = True
            self.frames = []
            self.master_map_fn = None
            self.analysis_fn = None
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def stop(self):
        with self.lock:
            self.active = False

    def _run(self):
        print("[Mapping] Session Started")
        while True:
            with self.lock:
                if not self.active: break
            
            try:
                # Use fast=true to avoid camera restarts
                resp = requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot?fast=true", timeout=5)
                if resp.status_code == 200:
                    ts = int(time.time() * 100)
                    fn = f"map_frame_{ts}.jpg"
                    path = os.path.join(CAPTURE_DIR, fn)
                    with open(path, 'wb') as f: f.write(resp.content)
                    
                    with self.lock:
                        self.frames.append(path)
                        if len(self.frames) >= 2:
                            stitched_fn = "map_realtime.jpg"
                            stitched_path = os.path.join(CAPTURE_DIR, stitched_fn)
                            
                            # Use last 4 frames for the rolling map
                            subset = self.frames[-4:]
                            success, msg = stitch_images(subset, stitched_path)
                            
                            if success:
                                self.master_map_fn = stitched_fn
                                # Only analyze if frame count is a multiple of 4 to avoid overload
                                if len(self.frames) % 4 == 0:
                                    # Use fast_mode=True for analysis in mapping
                                    result = analyze_image(stitched_path, output_dir=CAPTURE_DIR, fast_mode=True)
                                    self.analysis_fn = os.path.basename(result['output_path'])
                                else:
                                    self.analysis_fn = stitched_fn
            except Exception as e:
                # print(f"[Mapping] Snapshot Failed: {e}")
                pass
            
            time.sleep(2.0) # Sane polling rate
        print("[Mapping] Session Stopped")

mapping_session = MappingSession()
task_progress = {}

def update_task_progress(task_id, msg, p):
    task_progress[task_id] = {"msg": msg, "p": p, "ts": time.time()}

@app.route('/api/progress/<task_id>')
def get_progress(task_id):
    return jsonify(task_progress.get(task_id, {"msg": "Pending", "p": 0}))

def wifi_telemetry_thread():
    global telemetry_data
    while True:
        try:
            # Poll telemetry from the Pi over WiFi
            resp = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/telemetry", timeout=2)
            if resp.status_code == 200:
                new_data = resp.json()
                new_data["status"] = "ONLINE"
                
                # --- SMART INPUT VOLTAGE ESTIMATION ---
                power = new_data.get('power', {})
                if power.get('plugged'):
                    bat_v = float(power.get('voltage', 0))
                    bat_i = float(power.get('current', 0))
                    
                    if bat_v < 4.0 and bat_i < 0.1:
                        est_input = bat_v + 0.25
                    else:
                        est_input = 5.05
                    est_input = max(est_input, bat_v + 0.1)
                    new_data['power']['input_est'] = round(est_input, 2)
                    
                    if 'watts' not in new_data['power'] or new_data['power']['watts'] == 0:
                         new_data['power']['watts'] = round(est_input * max(0.05, bat_i), 2)
                else:
                    new_data['power']['input_est'] = 0.0
                    new_data['power']['watts'] = 0.0
                
                with data_lock: telemetry_data = new_data
            else:
                with data_lock: telemetry_data["status"] = "WIFI ERROR"
        except Exception as e:
            # print(f"[GS] WiFi Telemetry Error: {e}")
            with data_lock: telemetry_data["status"] = "OFFLINE"
        
        time.sleep(0.2)

threading.Thread(target=wifi_telemetry_thread, daemon=True).start()

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/telemetry')
def api_telemetry():
    with data_lock:
        data = telemetry_data.copy()
        with mapping_session.lock:
            data['mapping_status'] = 'active' if mapping_session.active else 'idle'
            data['mapping_frames'] = len(mapping_session.frames)
            data['mapping_image'] = mapping_session.analysis_fn
        return jsonify(data)

@app.route('/api/mapping/start', methods=['POST'])
def start_mapping():
    mapping_session.start()
    return jsonify({"status": "success"})

@app.route('/api/mapping/stop', methods=['POST'])
def stop_mapping():
    mapping_session.stop()
    return jsonify({"status": "success"})

@app.route('/api/upload', methods=['POST'])
def upload_and_analyze():
    task_id = request.args.get('task_id', 'upload')
    try:
        if 'file' not in request.files:
            return jsonify({"status": "error", "msg": "No file part"}), 400
        file = request.files['file']
        if file.filename == '':
            return jsonify({"status": "error", "msg": "No selected file"}), 400
            
        def cb(msg, p): update_task_progress(task_id, msg, p)
        cb("Uploading", 0.05)

        if file:
            filename = secure_filename(file.filename)
            filename = f"upload_{int(time.time())}_{filename}"
            save_path = os.path.join(CAPTURE_DIR, filename)
            file.save(save_path)
            
            # Immediately analyze
            result = analyze_image(save_path, output_dir=CAPTURE_DIR, progress_callback=cb)
            analysis_fn = os.path.basename(result['output_path'])
            
            return jsonify({
                "status": "success",
                "filename": filename,
                "analysis_file": analysis_fn,
                "analysis_url": f"/captures/{analysis_fn}",
                "sites": result['sites'],
                "zone_radius": result['zone_radius']
            })
    except Exception as e:
        print(f"[GS] Upload Failed: {e}")
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route('/telemetry_stream')
def telemetry_stream():
    def event_stream():
        while True:
            with data_lock:
                data = telemetry_data.copy()
                with mapping_session.lock:
                    data['mapping_status'] = 'active' if mapping_session.active else 'idle'
                    data['mapping_frames'] = len(mapping_session.frames)
                    data['mapping_image'] = mapping_session.analysis_fn
                json_data = json.dumps(data)
            yield f"data: {json_data}\n\n"
            time.sleep(0.25)
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/stream')
def proxy_video():
    try:
        req = requests.get(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/stream", stream=True, timeout=2)
        return Response(stream_with_context(req.iter_content(chunk_size=4096)), content_type=req.headers['Content-Type'])
    except: return "No Signal", 404

@app.route('/snapshot', methods=['POST'])
def proxy_snapshot():
    try:
        exposure = request.args.get('exposure', 0.0)
        resp = requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/snapshot", params={'exposure': exposure}, timeout=20)
        if resp.status_code == 200:
            timestamp = int(time.time())
            base_fn = f"snap_{timestamp}"
            img_fn = f"{base_fn}.jpg"
            with open(os.path.join(CAPTURE_DIR, img_fn), 'wb') as f: f.write(resp.content)
            pose = [float(x) for x in resp.headers.get('X-Pose', '1,0,0,0').split(',')]
            accel = [float(x) for x in resp.headers.get('X-Accel', '0,0,0').split(',')]
            metadata = {
                "image": img_fn, "timestamp": timestamp, "exposure_sec": float(exposure),
                "pose_quaternion_wxyz": pose, "accel_ms2": {"x":accel[0], "y":accel[1], "z":accel[2]}
            }
            with open(os.path.join(CAPTURE_DIR, f"{base_fn}.json"), 'w') as f: json.dump(metadata, f, indent=4)
            return jsonify({"status": "success", "file": img_fn, "pose": pose})
    except Exception as e: print(f"[GS] Snapshot Failed: {e}")
    return jsonify({"status": "error"}), 500

@app.route('/api/captures')
def list_captures():
    try:
        files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.lower().endswith(('.jpg', '.jpeg'))], reverse=True)
        return jsonify(files)
    except: return jsonify([])

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

@app.route('/api/analyze/<filename>', methods=['POST'])
def analyze_capture(filename):
    task_id = request.args.get('task_id', filename)
    try:
        image_path = os.path.join(CAPTURE_DIR, filename)
        if not os.path.exists(image_path):
            return jsonify({"status": "error", "msg": "File not found"}), 404
            
        def cb(msg, p): update_task_progress(task_id, msg, p)
        
        # Analysis
        result = analyze_image(image_path, output_dir=CAPTURE_DIR, progress_callback=cb)
        analysis_fn = os.path.basename(result['output_path'])
        
        return jsonify({
            "status": "success",
            "analysis_file": analysis_fn,
            "analysis_url": f"/captures/{analysis_fn}",
            "sites": result['sites'],
            "zone_radius": result['zone_radius']
        })
    except Exception as e:
        print(f"[GS] Analysis Failed: {e}")
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route('/api/stitch_and_analyze', methods=['POST'])
def stitch_and_analyze():
    task_id = request.args.get('task_id', 'stitch')
    try:
        filenames = request.json.get('filenames', [])
        print(f"[GS] Stitch Request: {len(filenames)} images")
        if not filenames or len(filenames) < 2:
            return jsonify({"status": "error", "msg": "Need at least 2 images"}), 400
            
        def cb(msg, p): 
            print(f"[GS] Stitch Progress: {msg} ({p*100:.0f}%)")
            update_task_progress(task_id, msg, p)
            
        cb("Stitching", 0.1)

        image_paths = [os.path.join(CAPTURE_DIR, f) for f in filenames]
        timestamp = int(time.time())
        stitched_fn = f"stitched_{timestamp}.jpg"
        stitched_path = os.path.join(CAPTURE_DIR, stitched_fn)
        
        print(f"[GS] Running OpenCV Stitcher on {len(image_paths)} files...")
        success, msg = stitch_images(image_paths, stitched_path)
        if not success:
            print(f"[GS] Stitching Failed: {msg}")
            return jsonify({"status": "error", "msg": f"Stitching failed: {msg}"}), 500
            
        print(f"[GS] Stitch Success! -> {stitched_fn}. Analyzing...")
        # Now analyze the stitched image
        result = analyze_image(stitched_path, output_dir=CAPTURE_DIR, progress_callback=cb)
        analysis_fn = os.path.basename(result['output_path'])
        
        print(f"[GS] Analysis Complete -> {analysis_fn}")
        return jsonify({
            "status": "success",
            "stitched_file": stitched_fn,
            "analysis_file": analysis_fn,
            "analysis_url": f"/captures/{analysis_fn}",
            "sites": result['sites'],
            "zone_radius": result['zone_radius']
        })
    except Exception as e:
        print(f"[GS] Stitch and Analyze CRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route('/api/clock', methods=['POST'])
def proxy_clock():
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/clock", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

@app.route('/api/i2c', methods=['POST'])
def proxy_i2c():
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/i2c", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

@app.route('/api/power', methods=['POST'])
def proxy_power():
    try:
        requests.post(f"http://{PI_WIFI_IP}:{PI_VIDEO_PORT}/api/power", json=request.json, timeout=3)
        return jsonify({"status": "proxied"})
    except Exception as e: return jsonify({"status": "error", "msg": str(e)}), 502

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
