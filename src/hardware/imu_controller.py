import time
import threading
import math
import smbus2


class IMUController:
    def __init__(self, address=0x68, bus_number=1):
        self.address = address
        self.bus_number = bus_number
        self.available = False
        self.stop_event = threading.Event()
        self.last_orientation = 0.0
        self.orientation_lock = threading.Lock()

        try:
            self.bus = smbus2.SMBus(bus_number)
            # Проверяем, отвечает ли устройство по адресу
            self.bus.read_byte_data(self.address, 0x75)  # WHO_AM_I register для MPU-6050
            self._initialize()
            self.imu_thread = threading.Thread(target=self._imu_reader, daemon=True)
            self.imu_thread.start()
            self.available = True
            print(f"[IMU] Initialized on bus {bus_number}, address 0x{address:02X}")
        except Exception as e:
            if hasattr(self, 'bus'):
                self.bus.close()
            print(f"[IMU] Not available ({e}). Running in STUB mode (orientation = 0).")

    def _initialize(self):
        # Разбудить MPU-6050 (снять sleep)
        self.bus.write_byte_data(self.address, 0x6B, 0)
        time.sleep(0.1)

    def _imu_reader(self):
        last_time = time.monotonic()
        while not self.stop_event.is_set():
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
                gyro_z = (data[10] << 8) | data[11]
                if gyro_z > 32767:
                    gyro_z -= 65536
                now = time.monotonic()
                dt = min(now-last_time, 0.1)
                last_time = now
                with self.orientation_lock:
                    self.last_orientation += math.radians(gyro_z / 131.0) * dt
                time.sleep(0.01)
            except Exception as e:
                print(f"[IMU] Read error: {e}")
                time.sleep(0.1)

    def get_orientation(self):
        if not self.available:
            return 0.0
        with self.orientation_lock:
            return self.last_orientation

    def reset_orientation(self):
        if not self.available:
            return
        with self.orientation_lock:
            self.last_orientation = 0.0

    def shutdown(self):
        if not self.available:
            print("[IMU] Shutdown (stub, nothing to do)")
            return
        self.stop_event.set()
        if self.imu_thread.is_alive():
            self.imu_thread.join(timeout=2.0)
        try:
            self.bus.close()
        except Exception:
            pass
        print("[IMU] Shutdown")
