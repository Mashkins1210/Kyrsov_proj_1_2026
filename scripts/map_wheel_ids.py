import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.utils.config import DXL_IDS, ADDR_MX_TORQUE_ENABLE

motor = MotorController()

for dxl_id in DXL_IDS:
    motor.packetHandler.write1ByteTxRx(
        motor.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0
    )

baseline = {dxl_id: motor.read_position(dxl_id) for dxl_id in DXL_IDS}
print(f"[BASELINE] {baseline}\n")
print("=" * 70)
print("Крутите РОВНО ОДНО колесо на ПОЛНЫЙ ОБОРОТ. Смотрите, какой ID меняется.")
print("=" * 70)

try:
    while True:
        current = {dxl_id: motor.read_position(dxl_id) for dxl_id in DXL_IDS}
        deltas = {k: current[k] - baseline[k] for k in current}
        line = "  ".join(f"ID{k}:{d:+7d}" for k, d in sorted(deltas.items()))
        print(f"  {line}")
        time.sleep(0.7)
except KeyboardInterrupt:
    print("\n[TEST] Interrupted")
finally:
    motor.shutdown()