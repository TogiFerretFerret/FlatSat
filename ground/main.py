import cv2
import sys
import time
import threading

# We use the Ultralytics library for YOLOv8 (State of the art, easy to use)
try:
    from ultralytics import YOLO
except ImportError:
    print("Missing library! Please run: pip install ultralytics")
    sys.exit(1)

# --- CONFIGURATION ---
# REPLACE THIS with your Pi's actual IP address shown in the Pi's terminal
# Example: "http://192.168.1.183:8000/video_feed"
STREAM_URL = "http://cubesat.local:8000/video_feed" 

class ThreadedStream:
    """
    Decouples Video Reading from Frame Processing.
    This prevents the ML inference time from blocking the network buffer,
    ensuring we always process the LATEST frame (Low Latency).
    """
    def __init__(self, src):
        self.capture = cv2.VideoCapture(src)
        self.lock = threading.Lock()
        self.frame = None
        self.status = False
        self.running = True
        
        if self.capture.isOpened():
            self.status = True
            # Start the background thread
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()
        else:
            print("Failed to open stream in ThreadedStream.")

    def update(self):
        while self.running:
            if self.capture.isOpened():
                status, frame = self.capture.read()
                with self.lock:
                    self.status = status
                    self.frame = frame
                # If we read too fast, we spinlock CPU, but for MJPEG it blocks on socket read
            else:
                time.sleep(0.1)

    def get_latest_frame(self):
        with self.lock:
            return self.status, self.frame

    def stop(self):
        self.running = False
        if self.capture.isOpened():
            self.capture.release()

def main():
    print("=========================================")
    print("   ARTEMIS GROUND STATION :: ML LAYER    ")
    print("=========================================")

    # 1. Load the Model
    # 'yolov8n.pt' is the "Nano" model. It is the smallest and fastest version.
    # It will automatically download the weights (approx 6MB) on the first run.
    print("[System] Loading Neural Network (YOLOv8 Nano)...")
    model = YOLO('yolov8n.pt')

    # 2. Connect to Video Stream (Threaded)
    print(f"[System] Connecting to Uplink: {STREAM_URL}")
    stream = ThreadedStream(STREAM_URL)
    
    # Allow a moment for the thread to fill the buffer
    time.sleep(1.0)

    if not stream.status:
        print("CRITICAL: Could not connect to video stream.")
        print("Troubleshooting:")
        print("1. Is the Pi script running?")
        print("2. Is the IP address in STREAM_URL correct?")
        print("3. Are both devices on the same Wi-Fi?")
        return

    print("[System] Link Established. Starting Inference Loop...")
    print("Press 'q' to quit.")

    # Variables for FPS calculation
    prev_frame_time = 0
    new_frame_time = 0

    while True:
        # 3. Get Latest Frame (Non-Blocking)
        success, frame = stream.get_latest_frame()
        
        if not success or frame is None:
            # Wait briefly if no frame yet
            time.sleep(0.01)
            continue

        # 4. Run Machine Learning (Inference)
        # stream=True optimizes memory for video sources
        # verbose=False stops it from spamming the terminal with "Person detected" logs
        results = model(frame, verbose=False)

        # 5. Visualize & Annotate
        # Loop through results (usually just one per frame)
        for result in results:
            # result.plot() draws the bounding boxes and labels automatically
            annotated_frame = result.plot()
            
            # Calculate FPS to monitor performance
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
            prev_frame_time = new_frame_time
            
            # Add FPS counter to top-left corner
            cv2.putText(annotated_frame, f"FPS: {int(fps)}", (7, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 2, cv2.LINE_AA)

            # Display the resulting frame in a window
            cv2.imshow("Artemis Live Feed + Object Detection", annotated_frame)

        # 6. Exit Logic (Press 'q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    stream.stop()
    cv2.destroyAllWindows()
    print("[System] Offline.")

if __name__ == "__main__":
    main()
