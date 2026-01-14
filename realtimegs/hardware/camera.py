import time
import io
try:
    from picamera2 import Picamera2
except ImportError:
    print("CRITICAL: 'picamera2' not found. Did you run 'uv venv --system-site-packages'?")
    raise

class CameraSystem:
    def __init__(self):
        print("[Hardware] Initializing Optical Sensor...")
        self.picam2 = Picamera2()
        
        # Configure Camera
        # Note: 640x480 is set for smooth streaming. 
        # For better Photogrammetry, you might want to increase this (e.g., 1920x1080),
        # but it may slow down the web stream.
        config = self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"} 
        )
        self.picam2.configure(config)
        self.picam2.start()
        print("[Hardware] Camera Active.")

    def get_streaming_frame(self):
        """
        Grabs a generic JPEG frame for the web browser.
        """
        try:
            # Create a stream in memory (RAM) instead of saving to disk
            stream = io.BytesIO()
            
            # Capture jpeg to the stream. 
            # FIXED: Removed 'use_video_port' which caused the crash.
            # Picamera2 automatically handles the stream via the configuration.
            self.picam2.capture_file(stream, format="jpeg")
            
            # Rewind the stream to the beginning so we can read it
            stream.seek(0)
            return stream.read()
        except Exception as e:
            print(f"[Camera] Stream Error: {e}")
            return None

    def capture_high_res(self, filepath):
        """
        Captures a HIGH QUALITY image to the disk.
        """
        try:
            self.picam2.capture_file(filepath)
            return True
        except Exception as e:
            print(f"[Camera] Capture Error: {e}")
            return False

    def stop(self):
        print("[Hardware] Stopping Camera...")
        self.picam2.stop()
        self.picam2.close()
