import sys
import os
import time
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.control.odometry import Odometry
from src.utils import config

motor = MotorController()
odom = Odometry(
    motor_controller=motor,
    left_motors=[2, 9],
    right_motors=[7, 8],
    wheel_diameter_mm=config.WHEEL_DIAMETER_MM,
    wheelbase_mm=config.WHEELBASE_MM,
    ticks_per_rev=config.TICKS_PER_REV,
    update_hz=20
)

print("\n[TEST] Робот на подставке. Моторы сами крутят колёса.\n")

def show_pose(label):
    x, y, th = odom.get_pose_meters()
    th_deg = math.degrees(th)
    print(f"  [{label:20s}]  x={x:+.3f}m  y={y:+.3f}m  theta={th:+.2f}rad ({th_deg:+6.1f}°)")

# Сброс одометрии
odom.reset(0.0, 0.0, 0.0)
time.sleep(0.5)
show_pose("start")

# --- Вперёд 2 секунды ---
print("\n[ACTION] Двигаюсь ВПЕРЁД 2 секунды...")
motor.move_forward(config.BASE_SPEED)
time.sleep(2.0)
motor.stop_all()
time.sleep(0.5)
show_pose("after forward 2s")

# --- Влево 1 секунду ---
print("\n[ACTION] Поворот ВЛЕВО 1 секунда...")
motor.rotate_left(config.ROTATE_SPEED)
time.sleep(1.0)
motor.stop_all()
time.sleep(0.5)
show_pose("after turn left 1s")

# --- Вправо 1 секунду ---
print("\n[ACTION] Поворот ВПРАВО 1 секунда...")
motor.rotate_right(config.ROTATE_SPEED)
time.sleep(1.0)
motor.stop_all()
time.sleep(0.5)
show_pose("after turn right 1s")

odom.shutdown()
motor.shutdown()