#!/usr/bin/env python3
import cv2
import time
import threading
from queue import Queue, Empty

class QRCamera:
    def __init__(self, camera_index=0, detect_every=3, scale=0.5):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            print(f"Error: Cannot open camera {camera_index}")
            self.cap = None
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.qr_detector = cv2.QRCodeDetector()
        self.stop_event = threading.Event()

        self.frame_queue = Queue(maxsize=1)
        self.display_queue = Queue(maxsize=1)

        self.detected_codes = []
        self.last_code = None
        self.last_code_time = 0

        self.detect_every = detect_every
        self.scale = scale
        self.frame_count = 0
        self.fps = 0
        self.fps_time = time.time()
        self.fps_frames = 0

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)

        self.reader_thread.start()
        self.process_thread.start()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            if self.cap is None or not self.cap.isOpened():
                break

            ret, frame = self.cap.read()
            if not ret:
                continue

            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass

            self.frame_queue.put(frame)

    def _process_loop(self):
        while not self.stop_event.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except Empty:
                continue

            self.frame_count += 1
            self.fps_frames += 1

            now = time.time()
            if now - self.fps_time >= 1.0:
                self.fps = self.fps_frames
                self.fps_frames = 0
                self.fps_time = now

            display_frame = frame.copy()
            should_detect = (self.frame_count % self.detect_every == 0)

            if should_detect:
                small = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                                   interpolation=cv2.INTER_AREA)
                data, bbox, _ = self.qr_detector.detectAndDecode(small)

                if data:
                    if data != self.last_code or (now - self.last_code_time) > 2.0:
                        self.last_code = data
                        self.last_code_time = now
                        if data not in self.detected_codes:
                            self.detected_codes.append(data)
                        print(f"[{time.strftime('%H:%M:%S')}] QR: '{data}' "
                              f"(total: {len(self.detected_codes)})")

                    if bbox is not None:
                        pts = (bbox[0] / self.scale).astype(int)
                        for i in range(4):
                            cv2.line(display_frame, tuple(pts[i]),
                                     tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)
                        cv2.putText(display_frame, data,
                                    (pts[0][0], max(pts[0][1] - 5, 15)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.putText(display_frame, f"FPS: {self.fps}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            if self.last_code:
                cv2.putText(display_frame, f"Last: {self.last_code[:25]}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 255), 1)

            if not self.display_queue.empty():
                try:
                    self.display_queue.get_nowait()
                except Empty:
                    pass
            self.display_queue.put(display_frame)

    def show(self):
        print("Camera opened. Controls: q=quit, s=save, c=clear")
        print("-" * 50)

        while not self.stop_event.is_set():
            try:
                frame = self.display_queue.get(timeout=0.1)
            except Empty:
                continue

            cv2.imshow('QR Code Test', frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nQuit requested")
                break
            elif key == ord('s'):
                filename = f"qr_frame_{int(time.time())}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved: {filename}")
            elif key == 27:
                break

    def shutdown(self):
        self.stop_event.set()
        self.reader_thread.join(timeout=2.0)
        self.process_thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

        print("\n" + "=" * 50)
        print(f"Total frames: {self.frame_count}")
        print(f"Unique QR codes: {len(self.detected_codes)}")
        for i, code in enumerate(self.detected_codes, 1):
            print(f"  {i}. '{code}'")
        print("=" * 50)

if __name__ == "__main__":
    import sys

    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"Invalid camera index: {sys.argv[1]}")
            sys.exit(1)

    print("QR Code Camera Test (optimized)")
    print(f"Camera index: {camera_index}")
    print()

    cam = QRCamera(camera_index=camera_index, detect_every=3, scale=0.5)
    if cam.cap is None:
        sys.exit(1)

    try:
        cam.show()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        cam.shutdown()