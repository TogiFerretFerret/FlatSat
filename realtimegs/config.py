import os

# --- NETWORK ---
HTTP_PORT = 8000

# --- SENSOR TUNING ---
# Acceleration Threshold (m/s^2). Gravity is 9.8.
SHAKE_THRESHOLD = 15.0  

# --- DATA STORAGE ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")

if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)
