import time
import threading
import smbus2

class IMUController:
    def __init__(self, address=0x68):
        self.bus = smbus2.SMBus(1)
        self.address = address
        self.stop_event = threading.Event()
        self.last_orientation = 0.0
        self.orientation_lock = threading.Lock()
        
        self._initialize()
        self.imu_thread = threading.Thread(target=self._imu_reader, daemon=True)
        self.imu_thread.start()
        print("IMU initialized")

    def _initialize(self):
        self.bus.write_byte_data(self.address, 0x6B, 0)
        time.sleep(0.1)

    def _imu_reader(self):
        while not self.stop_event.is_set():
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
                gyro_z = (data[10] << 8) | data[11]
                if gyro_z > 32767:
                    gyro_z -= 65536
                
                with self.orientation_lock:
                    self.last_orientation += gyro_z * 0.001
                
                time.sleep(0.01)
            except Exception as e:
                print(f"IMU error: {e}")
                time.sleep(0.1)

    def get_orientation(self):
        with self.orientation_lock:
            return self.last_orientation

    def reset_orientation(self):
        with self.orientation_lock:
            self.last_orientation = 0.0

    def shutdown(self):
        self.stop_event.set()
        if self.imu_thread.is_alive():
            self.imu_thread.join(timeout=2.0)
        print("IMU shutdown")