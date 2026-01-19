import threading
import time
from collections import deque
# custom power driver for pisugar 3
try:
    import smbus
except ImportError:
    smbus = None

# --- CONFIG FOR PISUGAR 3 ONLY ---
PISUGAR_ADDR = 0x57 

# Discharge curve specifically for PiSugar 3 (4.2V max)
BATTERY_CURVE = [
    (4.2, 100.0), 
    (4.0, 80.0), 
    (3.7, 60.0), 
    (3.5, 20.0), 
    (3.1, 0.0)
]

class PiSugar:
    def __init__(self):
        self.ready = False
        self.model = "PiSugar3"
        self.bus = None
        
        # State
        self.battery_voltage = 0.00
        self.voltage_history = deque(maxlen=10)
        self.battery_level = 0.0
        self.temperature = 0
        self.power_plugged = False
        self.allow_charging = True
        self.i2creg = []
        
        if smbus:
            try:
                self.bus = smbus.SMBus(1)
                # Start background polling
                self.thread = threading.Thread(target=self._poll_loop, daemon=True)
                self.thread.start()
            except Exception as e:
                print(f"[Power] SMBus Error: {e}")
        else:
            print("[Power] SMBus not found (Simulation Mode)")

    def get_data(self):
        """Returns power stats for Telemetry."""
        return {
            "model": self.model,
            "voltage": round(self.battery_voltage, 2),
            "level": round(self.battery_level, 1),
            "plugged": self.power_plugged,
            "charging": self.allow_charging and self.power_plugged,
            "temp": round(self.temperature, 1),
            "status": "ONLINE" if self.ready else "SEARCHING"
        }

    def _poll_loop(self):
        """Background loop: Connect once, then poll forever."""
        
        # 1. Connection Phase
        while not self.ready:
            try:
                # Try to read a dummy byte to confirm presence
                self.bus.read_byte_data(PISUGAR_ADDR, 0x00)
                self.ready = True
                print(f"[Power] Connected to {self.model} at 0x{PISUGAR_ADDR:02X}")
            except:
                time.sleep(2) # Retry delay

        # 2. Polling Phase
        while True:
            try:
                # FIX: Read in chunks of 32 bytes (SMBus Limit)
                # We need up to register 0x23 (Voltage), so 0x00-0x2F is sufficient.
                
                # Chunk 1: 0x00 - 0x1F (32 bytes)
                chunk1 = self.bus.read_i2c_block_data(PISUGAR_ADDR, 0, 32)
                # Chunk 2: 0x20 - 0x2F (16 bytes)
                chunk2 = self.bus.read_i2c_block_data(PISUGAR_ADDR, 32, 16)
                
                self.i2creg = chunk1 + chunk2
                
                self._parse_data()
                
                time.sleep(1.0) # Update rate
            except Exception as e:
                print(f"[Power] Read Error: {e}")
                time.sleep(2) # Back off on error

    def _parse_data(self):
        if not self.i2creg or len(self.i2creg) < 36: return

        # 1. Voltage (High byte at 0x22, Low byte at 0x23)
        # Note: 0x22 is index 34 in decimal, 0x23 is index 35
        high = self.i2creg[0x22]
        low = self.i2creg[0x23]
        self.battery_voltage = ((high << 8) + low) / 1000.0

        # 2. Temperature (0x04, offset by -40)
        self.temperature = self.i2creg[0x04] - 40

        # 3. Status Flags (0x02)
        # Bit 7: Power Plugged
        # Bit 6: Allow Charging
        ctr1 = self.i2creg[0x02]
        self.power_plugged = (ctr1 & (1 << 7)) != 0
        self.allow_charging = (ctr1 & (1 << 6)) != 0

        # 4. Calculate Percentage
        self._calc_level()

    def _calc_level(self):
        self.voltage_history.append(self.battery_voltage)
        
        # Smooth voltage (simple moving average)
        if len(self.voltage_history) < 3:
            avg_v = sum(self.voltage_history) / len(self.voltage_history)
        else:
            # Drop outliers (min/max) for stability
            sorted_v = sorted(self.voltage_history)
            avg_v = sum(sorted_v[1:-1]) / (len(sorted_v) - 2)

        # Interpolate Curve
        level = 0.0
        curve = BATTERY_CURVE
        
        if avg_v >= curve[0][0]: 
            level = 100.0
        elif avg_v <= curve[-1][0]:
            level = 0.0
        else:
            for i in range(len(curve) - 1):
                v_high, p_high = curve[i]
                v_low, p_low = curve[i+1]
                if v_low <= avg_v <= v_high:
                    # Linear interpolation
                    level = p_low + (p_high - p_low) * (avg_v - v_low) / (v_high - v_low)
                    break
        
        self.battery_level = level
