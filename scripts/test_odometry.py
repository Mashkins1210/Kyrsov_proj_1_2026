import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hardware.motor_controller import MotorController
from src.control.odometry import Odometry
from src.utils import config


def main():
    print("=" * 50)
    print("=== ODOMETRY TEST ===")
    print("=" * 50)

    # --- Инициализация моторов ---
    print("\n[1] Initializing motors...")
    motor = MotorController()

    # --- Инициализация одометрии ---
    print("\n[2] Initializing odometry...")
    odom = Odometry(
        motor_controller=motor,
        left_motors=[2, 9],
        right_motors=[7, 8],
        wheel_diameter_mm=config.WHEEL_DIAMETER_MM,
        wheelbase_mm=config.WHEELBASE_MM,
        ticks_per_rev=config.TICKS_PER_REV,
        update_hz=20
    )

    # --- Инструкции ---
    print("\n" + "=" * 50)
    print("ИНСТРУКЦИЯ:")
    print("  1. Моторы ВЫКЛЮЧЕНЫ (не подавайте 12В).")
    print("  2. Толкайте робота руками ВПЕРЁД на 50 см.")
    print("  3. Затем поверните робота на месте на 90°.")
    print("  4. Следите за показаниями x, y, theta ниже.")
    print("=" * 50 + "\n")

    # --- Показываем позу каждую секунду ---
    try:
        for i in range(60):  # 60 секунд
            x, y, theta = odom.get_pose_meters()
            theta_deg = theta * 180.0 / 3.14159
            print(
                f"t={i:2d}s  |  "
                f"x={x:+.3f} m  |  "
                f"y={y:+.3f} m  |  "
                f"theta={theta:+.3f} rad ({theta_deg:+6.1f}°)"
            )
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[TEST] Interrupted by user")

    finally:
        print("\n[TEST] Shutting down...")
        try:
            odom.shutdown()
        except Exception as e:
            print(f"  [ODOM shutdown err] {e}")
        try:
            motor.shutdown()
        except Exception as e:
            print(f"  [Motor shutdown err] {e}")
        print("[TEST] Done")


if __name__ == "__main__":
    main()