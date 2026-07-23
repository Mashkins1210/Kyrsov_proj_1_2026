import time
import threading
from rplidar import RPLidar
from src.utils.config import LIDAR_DEVICE_PATH, LIDAR_BAUD_RATE, LIDAR_TIMEOUT

class LidarController:
    def __init__(self):
        self.lidar = RPLidar(
            port=LIDAR_DEVICE_PATH,
            baudrate=LIDAR_BAUD_RATE,
            timeout=LIDAR_TIMEOUT
        )
        self.lidar.start_motor()
        print("RPLidar started")
        
        self.stop_event = threading.Event()
        self.last_scan = None
        self.scan_lock = threading.Lock()
        self.scan_available = False
        
        self.lidar_thread = threading.Thread(
            target=self._lidar_reader, daemon=True
        )
        self.lidar_thread.start()
        print("Lidar reader thread started")

    def _lidar_reader(self):
        while not self.stop_event.is_set():
            try:
                for scan in self.lidar.iter_scans(max_buf_meas=4000):
                    if self.stop_event.is_set():
                        break
                    with self.scan_lock:
                        self.last_scan = scan
                        self.scan_available = True
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"Lidar error: {e}")
                    time.sleep(1.0)

    def get_fresh_scan(self, timeout=0.1):
        with self.scan_lock:
            if self.scan_available:
                self.scan_available = False
                return self.last_scan
            return self.last_scan

    def shutdown(self):
        self.stop_event.set()
        if self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=2.0)
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("Lidar disconnected")
        except Exception as e:
            print(f"Lidar shutdown error: {e}")