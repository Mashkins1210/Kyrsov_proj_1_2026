import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.utils.config import DXL_IDS, ADDR_MX_TORQUE_ENABLE

motor = MotorController()

print("\n=== DISABLING TORQUE (free-wheel mode) ===")
for dxl_id in DXL_IDS:
    comm, err = motor.packetHandler.write1ByteTxRx(
        motor.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0
    )
    print(f"  Motor {dxl_id}: torque disabled (comm={comm})")

print("\n=== READING RAW POSITIONS EVERY 0.5s ===")
print("Толкайте робота руками. Смотрите, меняются ли числа.\n")

try:
    for i in range(40):
        pos = motor.read_positions()
        print(f"t={i*0.5:5.1f}s  positions={pos}")
        time.sleep(0.5)
finally:
    motor.shutdown()