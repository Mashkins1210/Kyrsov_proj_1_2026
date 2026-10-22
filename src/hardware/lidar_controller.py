import threading
import time
import math
from rplidar import RPLidar
from src.utils.config import LIDAR_DEVICE_PATH, LIDAR_BAUD_RATE, LIDAR_TIMEOUT


class LidarController:
    def __init__(self, max_age=0.5):
        self.stop_event = threading.Event()
        self.scan_lock = threading.Condition()
        self.last_scan = None
        self.last_scan_at = 0.0
        self.max_age = max_age
        self.error = None
        self.lidar = RPLidar(port=LIDAR_DEVICE_PATH, baudrate=LIDAR_BAUD_RATE, timeout=LIDAR_TIMEOUT)
        self.lidar.start_motor()
        self.lidar_thread = threading.Thread(target=self._lidar_reader, daemon=True)
        self.lidar_thread.start()

    def _lidar_reader(self):
        while not self.stop_event.is_set():
            try:
                for scan_index, scan in enumerate(self.lidar.iter_scans(max_buf_meas=4000)):
                    if self.stop_event.is_set():
                        break
                    # The first batch starts mid-revolution; it is not a full scan.
                    if scan_index == 0:
                        continue
                    valid = [(q, a, d) for q, a, d in scan if q > 0 and math.isfinite(a) and math.isfinite(d) and 10 < d <= 12000]
                    with self.scan_lock:
                        self.last_scan = valid if len(valid) >= 30 else None
                        self.last_scan_at = time.monotonic()
                        self.error = None if self.last_scan else 'Insufficient valid measurements'
                        self.scan_lock.notify_all()
            except Exception as exc:
                with self.scan_lock:
                    self.last_scan = None
                    self.error = str(exc)
                    self.scan_lock.notify_all()
                self.stop_event.wait(0.5)

    def get_fresh_scan(self, timeout=0.1):
        deadline = time.monotonic()+max(0, timeout)
        with self.scan_lock:
            while not self.stop_event.is_set():
                if self.last_scan is not None and time.monotonic()-self.last_scan_at <= self.max_age:
                    return list(self.last_scan)
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return None
                self.scan_lock.wait(remaining)
            return None

    def shutdown(self):
        self.stop_event.set()
        with self.scan_lock:
            self.scan_lock.notify_all()
        for operation in (self.lidar.stop, self.lidar.stop_motor, self.lidar.disconnect):
            try:
                operation()
            except Exception:
                pass
        self.lidar_thread.join(timeout=2)
