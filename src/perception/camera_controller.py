import cv2
import threading
from queue import Queue, Empty

class CameraController:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            print(f"Failed to open camera {camera_index}")
            self.cap = None
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print(f"Camera {camera_index} opened successfully")

        self.qr_detector = cv2.QRCodeDetector()
        self.stop_event = threading.Event()
        self.qr_queue = Queue(maxsize=5)
        self.last_frame = None
        self.frame_lock = threading.Lock()
        self.last_qr_data = None

        self.camera_thread = threading.Thread(
            target=self._camera_reader, daemon=True
        )
        self.camera_thread.start()
        print("Camera reader thread started")

    def _camera_reader(self):
        while not self.stop_event.is_set():
            if self.cap is None or not self.cap.isOpened():
                break

            ret, frame = self.cap.read()
            if not ret:
                continue

            with self.frame_lock:
                self.last_frame = frame.copy()

            data, bbox, _ = self.qr_detector.detectAndDecode(frame)

            if data:
                if data != self.last_qr_data:
                    self.last_qr_data = data
                    print(f"QR code detected: {data}")

                    while not self.qr_queue.empty():
                        try:
                            self.qr_queue.get_nowait()
                        except Empty:
                            break

                    self.qr_queue.put({
                        'data': data,
                        'bbox': bbox
                    })

    def get_qr_code(self, timeout=0.05):
        try:
            return self.qr_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_last_qr_data(self):
        return self.last_qr_data

    def get_frame(self):
        with self.frame_lock:
            return self.last_frame.copy() if self.last_frame is not None else None

    def shutdown(self):
        try:
            self.stop_event.set()
            if self.camera_thread.is_alive():
                self.camera_thread.join(timeout=2.0)
            if self.cap is not None:
                self.cap.release()
            print("Camera disconnected")
        except Exception as e:
            print(f"Camera shutdown error: {e}")