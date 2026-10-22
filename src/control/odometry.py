import math
import threading
import time
from src.utils.config import WHEEL_FORWARD_SIGNS


class Odometry:
    def __init__(self, motor_controller, left_motors, right_motors,
                 wheel_diameter_mm, wheelbase_mm, ticks_per_rev=4096,
                 update_hz=10, encoder_signs=None, angular_scale=1.0):
        if wheel_diameter_mm <= 0 or wheelbase_mm <= 0 or update_hz <= 0:
            raise ValueError('Invalid calibrated wheel geometry')
        if not math.isfinite(angular_scale) or not 0.1 <= angular_scale <= 2.0:
            raise ValueError('Invalid angular calibration scale')
        self.angular_scale = angular_scale
        self.motor = motor_controller
        self.left_ids, self.right_ids = list(left_motors), list(right_motors)
        self.ticks_per_rev = ticks_per_rev
        self.wheel_circumference = math.pi*wheel_diameter_mm
        self.wheelbase_mm = wheelbase_mm
        self.encoder_signs = dict(WHEEL_FORWARD_SIGNS if encoder_signs is None else encoder_signs)
        self.x = self.y = self.theta = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._interval = 1/update_hz
        self.fault = None
        self._last_positions = self._read_all_positions()
        self.last_update = time.monotonic()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _read_all_positions(self):
        return {i: self.motor.read_position(i) for i in self.left_ids+self.right_ids}

    def _avg_delta(self, current, last, ids):
        return sum((current[i]-last[i]+self.ticks_per_rev/2) % self.ticks_per_rev-self.ticks_per_rev/2 for i in ids)/len(ids)

    def _update_loop(self):
        while not self._stop_event.wait(self._interval):
            try:
                current = self._read_all_positions()
                now = time.monotonic()
                if now-self.last_update > 0.5:
                    raise RuntimeError('Encoder sampling gap; position wrap is ambiguous')
                dl = sum(self._avg_delta(current, self._last_positions, [i])*self.encoder_signs[i] for i in self.left_ids)/len(self.left_ids)
                dr = sum(self._avg_delta(current, self._last_positions, [i])*self.encoder_signs[i] for i in self.right_ids)/len(self.right_ids)
                unit = self.wheel_circumference/self.ticks_per_rev
                distance = (dl+dr)*unit/2
                angle = (dr-dl)*unit/self.wheelbase_mm*self.angular_scale
                with self._lock:
                    self.x += distance*math.cos(self.theta+angle/2)
                    self.y += distance*math.sin(self.theta+angle/2)
                    self.theta = math.atan2(math.sin(self.theta+angle), math.cos(self.theta+angle))
                    self._last_positions = current
                    self.last_update = now
            except Exception as exc:
                self.fault = str(exc)
                try:
                    self.motor.stop_all()
                finally:
                    self._stop_event.set()

    def get_pose(self):
        with self._lock:
            if self.fault or time.monotonic()-self.last_update > 0.5:
                raise RuntimeError(self.fault or 'Odometry stale')
            return self.x, self.y, self.theta

    def get_pose_meters(self):
        x, y, theta = self.get_pose()
        return x/1000, y/1000, theta

    def reset(self, x=0.0, y=0.0, theta=0.0):
        with self._lock:
            self.x, self.y, self.theta = x, y, theta

    def shutdown(self):
        self._stop_event.set()
        self._thread.join(timeout=2)
