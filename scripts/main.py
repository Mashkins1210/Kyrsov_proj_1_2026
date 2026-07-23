import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.hardware.lidar_controller import LidarController
from src.perception.obstacle_detector import ObstacleDetector
from src.perception.camera_controller import CameraController
from src.control.state_machine import RobotStateMachine
from src.control.robot_controller import RobotController

class RobotApp:
    def __init__(self):
        print("Initializing Robot System (no visualization)...")
        self.motor_controller = MotorController()
        self.lidar_controller = LidarController()
        self.obstacle_detector = ObstacleDetector()
        self.camera_controller = CameraController(camera_index=0)
        self.state_machine = RobotStateMachine()
        self.robot_controller = RobotController(
            self.motor_controller,
            self.obstacle_detector,
            self.state_machine
        )
        self.is_running = True
        self.frame_count = 0
        self.last_log_time = time.time()
        print("Robot System Initialized")

    def check_qr_codes(self):
        qr_result = self.camera_controller.get_qr_code(timeout=0.01)
        if qr_result is not None:
            print(f"QR code detected: {qr_result['data']}")

    def run(self):
        print("Starting main loop...")
        try:
            while self.is_running:
                self.frame_count += 1
                
                self.check_qr_codes()

                scan = self.lidar_controller.get_fresh_scan(timeout=0.05)
                
                state = self.state_machine.get_state()

                if state == 'moving':
                    state_text, color, info = self.robot_controller.handle_moving_state(scan)
                elif state == 'wall_following':
                    state_text, color, info = self.robot_controller.handle_wall_following_state(scan)
                elif state == 'small_turn':
                    state_text, color, info = self.robot_controller.handle_small_turn_state(scan)
                elif state == 'turning':
                    state_text, color, info = self.robot_controller.handle_turning_state(scan)
                elif state == 'searching_wall':
                    state_text, color, info = self.robot_controller.handle_searching_wall_state(scan)
                else:
                    state_text, color, info = 'UNKNOWN', 'gray', f'Current: {state}'

                now = time.time()
                if now - self.last_log_time > 1.0:
                    fps = self.frame_count / (now - self.last_log_time)
                    print(f"Frame #{self.frame_count} | State: {state_text} | FPS: {fps:.1f} | {info}")
                    self.frame_count = 0
                    self.last_log_time = now

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received.")
        finally:
            self.shutdown()

    def shutdown(self):
        self.is_running = False
        print("\nShutting down systems...")
        self.motor_controller.shutdown()
        self.lidar_controller.shutdown()
        self.camera_controller.shutdown()
        print("All systems stopped cleanly.")

if __name__ == "__main__":
    app = RobotApp()
    app.run()