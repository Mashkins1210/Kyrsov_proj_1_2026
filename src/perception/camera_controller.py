import glob
import os
import threading
import time
from queue import Queue, Empty, Full
import cv2
cv2.setNumThreads(1)
import numpy as np


def discover_cameras():
    devices = {}
    for path in sorted(glob.glob('/dev/v4l/by-path/*-video-index0')):
        if 'usb-' in path or 'usbv2-' in path:
            devices.setdefault(os.path.realpath(path), path)
    return list(devices.values())


class CameraController:
    def __init__(self, camera_index=0, devices=None, required_count=3, max_age=1.0):
        self.devices = discover_cameras() if devices is None else list(devices)
        if len(self.devices) != required_count or len(set(map(os.path.realpath, self.devices))) != required_count:
            raise RuntimeError(f'Expected {required_count} distinct cameras, found {self.devices}')
        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.qr_queue = Queue(maxsize=32)
        self.last_qr_data = None
        self.max_age = max_age
        self.frames = {}
        self.errors = {}
        self.caps = {}
        self.capture_modes = {}
        self.threads = []
        self.available = False
        try:
            for index, device in enumerate(self.devices):
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                self.caps[index] = cap
                if not cap.isOpened():
                    raise RuntimeError(f'Cannot open camera {device}')
                # These Arducam units advertise MJPG only at 30 fps, but YUYV
                # 640x480 at 10 fps. Three cameras must use the supported rate.
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 10)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.capture_modes[index] = {'width':cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                    'height':cap.get(cv2.CAP_PROP_FRAME_HEIGHT), 'fps':cap.get(cv2.CAP_PROP_FPS),
                    'fourcc':int(cap.get(cv2.CAP_PROP_FOURCC))}
                if self.capture_modes[index]['fps'] != 10:
                    raise RuntimeError(f'Camera {device} rejected supported 10 fps mode')
            for index, cap in self.caps.items():
                t = threading.Thread(target=self._reader, args=(index, cap), daemon=True)
                self.threads.append(t)
                t.start()
            self.available = True
        except Exception:
            self.shutdown()
            raise

    def _reader(self, index, cap):
        detector = cv2.QRCodeDetector()
        seen = {}
        last_decode = 0.0
        while not self.stop_event.is_set():
            try:
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.errors[index] = 'Frame read failed'
                    self.stop_event.wait(0.1)
                    continue
                sample=frame[::4,::4]
                green=(sample[:,:,0]<3)&(sample[:,:,1]>80)&(sample[:,:,2]<3)
                if float(np.mean(green))>.25:
                    self.errors[index]='Corrupt solid-green frame'
                    self.stop_event.wait(0.1)
                    continue
                now = time.monotonic()
                with self.frame_lock:
                    self.frames[index] = (now, frame.copy())
                self.errors.pop(index, None)
                if now-last_decode < 0.2:
                    continue
                last_decode = now
                decoded = []
                if hasattr(detector, 'detectAndDecodeMulti'):
                    found, texts, points, _ = detector.detectAndDecodeMulti(frame)
                    if found:
                        decoded = list(zip(texts, points))
                if not decoded:
                    text, points, _ = detector.detectAndDecode(frame)
                    if text:
                        decoded = [(text, points)]
                for text, points in decoded:
                    if not text or len(text) > 4096 or now-seen.get(text, -10) < 1:
                        continue
                    seen[text] = now
                    self.last_qr_data = text
                    item = {'data': text, 'bbox': points, 'camera': index, 'timestamp': now}
                    try:
                        self.qr_queue.put_nowait(item)
                    except Full:
                        try:
                            self.qr_queue.get_nowait()
                        except Empty:
                            pass
                        self.qr_queue.put_nowait(item)
                self.stop_event.wait(0.02)
            except Exception as exc:
                self.errors[index] = str(exc)
                self.stop_event.wait(0.1)

    def get_qr_code(self, timeout=0.05):
        deadline = time.monotonic()+timeout
        while True:
            try:
                item = self.qr_queue.get(timeout=max(0, deadline-time.monotonic()))
            except Empty:
                return None
            if time.monotonic()-item['timestamp'] <= self.max_age:
                return item
            if time.monotonic() >= deadline:
                return None

    def get_last_qr_data(self):
        return self.last_qr_data

    def get_frames(self):
        with self.frame_lock:
            now = time.monotonic()
            return {i: frame.copy() for i, (stamp, frame) in self.frames.items() if now-stamp <= self.max_age}

    def get_frame(self, camera_index=0):
        return self.get_frames().get(camera_index)

    def shutdown(self):
        self.stop_event.set()
        for t in self.threads:
            t.join(timeout=2)
        for index, cap in self.caps.items():
            if index >= len(self.threads) or not self.threads[index].is_alive():
                cap.release()
        self.available = False

