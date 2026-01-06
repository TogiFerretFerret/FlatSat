import cv2
import sys
import time

# --- CONFIG ---
# If running this on the SAME machine (just outside WSL), use 'localhost'
# If running on a DIFFERENT machine, use the WSL host's IP address.
WSL_IP = "projectjanus" 
STREAM_URL = f"http://{WSL_IP}:5000/video_feed"

def main():
    print(f"Connecting to ML Stream at: {STREAM_URL}")
    
    cap = cv2.VideoCapture(STREAM_URL)
    
    if not cap.isOpened():
        print("Error: Could not connect to WSL stream.")
        print("1. Is ml_ground_station.py running inside WSL?")
        print("2. Is the Port 5000 forwarded? (Usually auto in WSL2)")
        return

    print("Stream started. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream interrupted.")
            break

        # Display in a Native Window
        cv2.imshow("Artemis Mission Control (Processed)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
