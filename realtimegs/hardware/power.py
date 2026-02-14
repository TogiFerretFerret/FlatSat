import threading
import time
from collections import deque

try:
    import smbus
except ImportError:
    smbus = None

# --- CONFIG FOR PISUGAR 3 ONLY ---
PISUGAR_ADDR = 0x57 

# Discharge curve for High Capacity (5000mAh/Plus) batteries
BATTERY_CURVE = [
    (4.10, 100.0), (4.05, 95.0), (3.90, 88.0), (3.80, 77.0), (3.70, 65.0),
    (3.62, 55.0), (3.58, 49.0), (3.49, 25.6), (3.32, 4.5), (3.1, 0.0),
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
                self.thread = threading.Thread(target=self._poll_loop, daemon=True)
                self.thread.start()
            except Exception as e:
                print(f"[Power] SMBus Error: {e}")
        else:
            print("[Power] SMBus not found (Simulation Mode)")

    def get_data(self):
        return {
            "model": self.model,
            "voltage": round(self.battery_voltage, 2),
            "level": round(self.battery_level, 1),
            "plugged": self.power_plugged,
            "charging": self.allow_charging and self.power_plugged,
            "allow_charging": self.allow_charging, 
            "temp": round(self.temperature, 1),
            "status": "ONLINE" if self.ready else "SEARCHING"
        }

    # --- CONTROL FUNCTIONS ---

    def power_cycle(self, delay=20):
        """
        Hard Power Cycle via System Watchdog.
        Use this to recover from stuck hardware (IMU, etc).
        1. Resets Watchdog Counter.
        2. Sets Timeout.
        3. Enables Watchdog.
        """
        if not self.ready or not self.bus: return False
        try:
            print(f"[Power] Setting Watchdog for Cycle in {delay}s...")
            # 1. Unlock
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x29)
            
            # 2. Reset Watchdog Counter (Bit 5 of 0x06)
            # This ensures we start with a clean slate
            ctrl = self.bus.read_byte_data(PISUGAR_ADDR, 0x06)
            self.bus.write_byte_data(PISUGAR_ADDR, 0x06, ctrl | (1 << 5)) 
            
            # 3. Set Timeout (Register 0x07)
            # PiSugar expects timeout / 2
            timeout_val = int(delay // 2)
            self.bus.write_byte_data(PISUGAR_ADDR, 0x07, timeout_val)
            
            # 4. Enable System Watchdog (Register 0x06, Bit 7)
            ctrl = self.bus.read_byte_data(PISUGAR_ADDR, 0x06)
            self.bus.write_byte_data(PISUGAR_ADDR, 0x06, ctrl | 0x80)
            
            # 5. Lock
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x00)
            return True
        except Exception as e:
            print(f"[Power] Watchdog Enable Failed: {e}")
            return False

    def shutdown(self, delay=10):
        """Cuts 5V power to the Pi (Hard Power Off)."""
        if not self.ready or not self.bus: return False
        try:
            print(f"[Power] Scheduling Power Cut in {delay}s...")
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x29) # Unlock
            self.bus.write_byte_data(PISUGAR_ADDR, 0x09, delay) # Delay
            
            ctrl = self.bus.read_byte_data(PISUGAR_ADDR, 0x02)
            new_ctrl = ctrl & ~(1 << 5) # Clear Bit 5 (Power Output)
            
            self.bus.write_byte_data(PISUGAR_ADDR, 0x02, new_ctrl)
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x00) # Lock
            return True
        except Exception as e:
            print(f"[Power] Power Cut Command Failed: {e}")
            return False

    def set_charging(self, enable):
        """Enables or Disables battery charging."""
        if not self.ready or not self.bus: return False
        try:
            print(f"[Power] Setting Charging: {enable}")
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x29) # Unlock
            
            ctrl = self.bus.read_byte_data(PISUGAR_ADDR, 0x02)
            if enable:
                new_ctrl = ctrl | (1 << 6) # Set bit 6
            else:
                new_ctrl = ctrl & ~(1 << 6) # Clear bit 6
            
            self.bus.write_byte_data(PISUGAR_ADDR, 0x02, new_ctrl)
            self.bus.write_byte_data(PISUGAR_ADDR, 0x0B, 0x00) # Lock
            
            self.allow_charging = enable
            return True
        except Exception as e:
            print(f"[Power] Charging Control Failed: {e}")
            return False

    def _poll_loop(self):
        while not self.ready:
            try:
                self.bus.read_byte_data(PISUGAR_ADDR, 0x00)
                self.ready = True
                print(f"[Power] Connected to {self.model} at 0x{PISUGAR_ADDR:02X}")
            except:
                time.sleep(2)

        while True:
            try:
                # Chunk 1: 0x00 - 0x1F (32 bytes)
                chunk1 = self.bus.read_i2c_block_data(PISUGAR_ADDR, 0, 32)
                # Chunk 2: 0x20 - 0x2F (16 bytes)
                chunk2 = self.bus.read_i2c_block_data(PISUGAR_ADDR, 32, 16)
                self.i2creg = chunk1 + chunk2
                self._parse_data()
                time.sleep(1.0)
            except Exception as e:
                print(f"[Power] Read Error: {e}")
                time.sleep(2)

    def _parse_data(self):
        if not self.i2creg or len(self.i2creg) < 36: return

        # Voltage
        high = self.i2creg[0x22]
        low = self.i2creg[0x23]
        self.battery_voltage = ((high << 8) + low) / 1000.0

        # Temp
        self.temperature = self.i2creg[0x04] - 40

        # Status
        ctr1 = self.i2creg[0x02]
        self.power_plugged = (ctr1 & (1 << 7)) != 0
        self.allow_charging = (ctr1 & (1 << 6)) != 0

        self._calc_level()

    def _calc_level(self):
        self.voltage_history.append(self.battery_voltage)
        if len(self.voltage_history) < 3:
            avg_v = sum(self.voltage_history) / len(self.voltage_history)
        else:
            sorted_v = sorted(self.voltage_history)
            avg_v = sum(sorted_v[1:-1]) / (len(sorted_v) - 2)

        level = 0.0
        curve = BATTERY_CURVE
        if avg_v >= curve[0][0]: level = 100.0
        elif avg_v <= curve[-1][0]: level = 0.0
        else:
            for i in range(len(curve) - 1):
                v_high, p_high = curve[i]
                v_low, p_low = curve[i+1]
                if v_low <= avg_v <= v_high:
                    level = p_low + (p_high - p_low) * (avg_v - v_low) / (v_high - v_low)
                    break
        self.battery_level = level
