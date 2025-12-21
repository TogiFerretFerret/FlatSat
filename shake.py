"""
The Python code you will write for this module should read
acceleration data from the IMU. When a reading comes in that surpasses
an acceleration threshold (indicating a shake), your Pi should pause,
trigger the camera to take a picture, then save the image with a
descriptive filename. You may use GitHub to upload your images automatically,
but for this activity it is not required.
"""

#AUTHOR: Your Name
#DATE: Today

#import libraries
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from picamera2 import Picamera2

#VARIABLES
THRESHOLD = 15.0     # Adjust this based on shake sensitivity (9.8 is gravity)
REPO_PATH = "."      # Use current directory for now
FOLDER_PATH = ""     # Leave empty if saving to same folder

# --- IMU INITIALIZATION ---
i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)

# --- CAMERA INITIALIZATION (Fixes "Device Busy" Error) ---
print("Initializing Camera...")
picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)
picam2.start()
print("Camera warmed up and ready.")
time.sleep(2)  # Wait for Auto-Exposure/White Balance

def getimg(filename="photo.jpg"):
    """
    Captures a photo using the globally active camera.
    Does NOT open/close the camera, so it is fast and safe for loops.
    """
    try:
        print(f"Capturing {filename}...")
        picam2.capture_file(filename)
        print("Image saved.")
    except Exception as e:
        print(f"Failed to capture image: {e}")

def git_push():
    """
    This function is complete. Stages, commits, and pushes new images to your GitHub repo.
    """
    pass

def img_gen(name):
    """
    This function is complete. Generates a new image name.
    """
    t = time.strftime("_%H%M%S")
    # Clean up path logic to avoid double slashes
    if FOLDER_PATH:
        imgname = (f'{REPO_PATH}/{FOLDER_PATH}/{name}{t}.jpg')
    else:
        imgname = (f'{name}{t}.jpg')
    return imgname

def take_photo():
    """
    Main loop to check accelerometer and take photos.
    """
    print("Ready to detect shakes...")
    
    # 1. Run the loop
    while True:
        # 2. Read Acceleration
        accel_x, accel_y, accel_z = accel_gyro.acceleration
        magnitude = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
        print("Magnitude:", magnitude)
        # 3. Check Threshold
        # (You can use magnitude > THRESHOLD or check axes individually)
        if magnitude > THRESHOLD:
            print(f"SHAKE DETECTED! (x:{accel_x:.1f} y:{accel_y:.1f} z:{accel_z:.1f})")
            
            # 4. Generate Name and Take Photo
            image_name = img_gen("FlatSat")
            getimg(image_name)
            
            # 5. Pause to prevent taking 50 photos for one shake
            time.sleep(2) 
        
        # Small delay to keep CPU usage low
        time.sleep(0.1)

def main():
    take_photo()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # This ALWAYS runs when you exit, releasing the camera
        print("Closing camera resources.")
        picam2.stop()
        picam2.close()
