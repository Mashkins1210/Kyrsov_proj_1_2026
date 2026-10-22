import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.utils.config import DXL_IDS, ADDR_MX_TORQUE_ENABLE

motor = MotorController()

# Отключаем torque
print("[SETUP] Torque off...")
for dxl_id in DXL_IDS:
    motor.packetHandler.write1ByteTxRx(
        motor.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0
    )

# Базовая позиция
baseline = {dxl_id: motor.read_position(dxl_id) for dxl_id in DXL_IDS}
print(f"[BASELINE] {baseline}\n")

print("=" * 60)
print("Крутите колёса ПО ОДНОМУ и смотрите, у какого мотора меняется счётчик:")
print("  - Сначала покрутите колесо ЛЕВОГО ПЕРЕДНЕГО — какой ID отреагировал?")
print("  - Потом ЛЕВОГО ЗАДНЕГО")
print("  - Потом ПРАВОГО ПЕРЕДНЕГО")
print("  - Потом ПРАВОГО ЗАДНЕГО")
print("=" * 60)
print("(Ctrl+C чтобы выйти)\n")

try:
    for i in range(120):
        current = {dxl_id: motor.read_position(dxl_id) for dxl_id in DXL_IDS}
        deltas = {k: current[k] - baseline[k] for k in current}
        line = "  ".join(f"ID{k}:{d:+6d}" for k, d in sorted(deltas.items()))
        print(f"t={i:3d}s  {line}")
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\n[TEST] Interrupted")
finally:
    motor.shutdown()