import pyrealsense2 as rs
import numpy as np
import threading

class RealSenseController:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.stop_event = threading.Event()
        self.last_depth_frame = None
        self.last_color_frame = None
        self.frame_lock = threading.Lock()
        
        self._initialize()
        self.rs_thread = threading.Thread(target=self._realsense_reader, daemon=True)
        self.rs_thread.start()
        print("RealSense initialized")

    def _initialize(self):
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

    def _realsense_reader(self):
        while not self.stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if depth_frame and color_frame:
                    with self.frame_lock:
                        self.last_depth_frame = np.asanyarray(depth_frame.get_data())
                        self.last_color_frame = np.asanyarray(color_frame.get_data())
            except Exception as e:
                print(f"RealSense error: {e}")
                time.sleep(0.1)

    def get_depth_at(self, x, y):
        with self.frame_lock:
            if self.last_depth_frame is not None:
                return self.last_depth_frame[y, x]
        return 0

    def get_color_frame(self):
        with self.frame_lock:
            return self.last_color_frame.copy() if self.last_color_frame is not None else None

    def get_depth_frame(self):
        with self.frame_lock:
            return self.last_depth_frame.copy() if self.last_depth_frame is not None else None

    def shutdown(self):
        self.stop_event.set()
        if self.rs_thread.is_alive():
            self.rs_thread.join(timeout=2.0)
        self.pipeline.stop()
        print("RealSense shutdown")