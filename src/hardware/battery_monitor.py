import time
import threading
import smbus2

class BatteryMonitor:
    def __init__(self, address=0x48):
        self.bus = smbus2.SMBus(1)
        self.address = address
        self.stop_event = threading.Event()
        self.last_voltage = 0.0
        self.voltage_lock = threading.Lock()
        self.low_battery_threshold = 10.5
        self.critical_battery_threshold = 10.0
        
        self.monitor_thread = threading.Thread(target=self._monitor, daemon=True)
        self.monitor_thread.start()
        print("Battery monitor initialized")

    def _monitor(self):
        while not self.stop_event.is_set():
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x00, 2)
                voltage = ((data[0] << 8) | data[1]) * 0.003
                
                with self.voltage_lock:
                    self.last_voltage = voltage
                
                if voltage < self.critical_battery_threshold:
                    print(f"CRITICAL BATTERY: {voltage:.2f}V")
                elif voltage < self.low_battery_threshold:
                    print(f"LOW BATTERY: {voltage:.2f}V")
                
                time.sleep(1.0)
            except Exception as e:
                print(f"Battery monitor error: {e}")
                time.sleep(1.0)

    def get_voltage(self):
        with self.voltage_lock:
            return self.last_voltage

    def is_low_battery(self):
        return self.last_voltage < self.low_battery_threshold

    def is_critical_battery(self):
        return self.last_voltage < self.critical_battery_threshold

    def shutdown(self):
        self.stop_event.set()
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        print("Battery monitor shutdown")