from dynamixel_sdk import *
from src.utils.config import (
    DXL_DEVICENAME, DXL_PROTOCOL_VERSION, DXL_BAUDRATE,
    DXL_IDS, ADDR_MX_TORQUE_ENABLE, ADDR_MX_GOAL_VELOCITY,
    ADDR_MX_PRESENT_VELOCITY
)

LEFT_MOTORS = [7, 8]
RIGHT_MOTORS = [2, 9]

class MotorController:
    def __init__(self):
        self.portHandler = PortHandler(DXL_DEVICENAME)
        self.packetHandler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_MX_GOAL_VELOCITY, 2
        )
        self.groupSyncRead = GroupSyncRead(
            self.portHandler, self.packetHandler,
            ADDR_MX_PRESENT_VELOCITY, 2
        )
        self.last_speed_command = 0
        self._initialize()

    def _initialize(self):
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(DXL_BAUDRATE):
            print("Failed to open DXL port")
            quit()
        for dxl_id in DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1
            )
            self.groupSyncRead.addParam(dxl_id)
        print("Dynamixels initialized")

    def set_velocities_bulk(self, velocities_dict):
        try:
            self.groupSyncWrite.clearParam()
            for dxl_id, velocity in velocities_dict.items():
                if dxl_id in LEFT_MOTORS:
                    velocity = -velocity
                velocity = max(min(velocity, 1023), -1023)
                if velocity < 0:
                    velocity = 1024 - velocity
                param = [DXL_LOBYTE(velocity), DXL_HIBYTE(velocity)]
                self.groupSyncWrite.addParam(dxl_id, param)
            result = self.groupSyncWrite.txPacket()
            if result != COMM_SUCCESS:
                print(f"Motor comm error: {self.packetHandler.getTxRxResult(result)}")
            else:
                self.last_speed_command = list(velocities_dict.values())[0] if velocities_dict else 0
        except Exception as e:
            print(f"Motor error: {e}")

    def stop_all(self):
        self.last_speed_command = 0
        self.set_velocities_bulk({dxl_id: 0 for dxl_id in DXL_IDS})

    def move_forward(self, speed):
        self.set_velocities_bulk({dxl_id: speed for dxl_id in DXL_IDS})

    def move_backward(self, speed):
        self.set_velocities_bulk({dxl_id: -speed for dxl_id in DXL_IDS})

    def rotate_left(self, speed):
        velocities = {2: -speed, 9: -speed, 7: speed, 8: speed}
        self.set_velocities_bulk(velocities)

    def rotate_right(self, speed):
        velocities = {2: speed, 9: speed, 7: -speed, 8: -speed}
        self.set_velocities_bulk(velocities)

    def move_forward_with_arc(self, direction, forward_speed, correction_speed):
        # ИСПРАВЛЕНО: ИНВЕРТИРОВАНЫ НАПРАВЛЕНИЯ
        if direction == 'left':
            # Чтобы дуга была влево, правые моторы должны быть медленнее
            left_speed = forward_speed + correction_speed  # Увеличиваем скорость левых
            right_speed = forward_speed - correction_speed # Уменьшаем скорость правых
        else: # direction == 'right'
            # Чтобы дуга была вправо, левые моторы должны быть медленнее
            left_speed = forward_speed - correction_speed # Уменьшаем скорость левых
            right_speed = forward_speed + correction_speed # Увеличиваем скорость правых

        left_speed = max(min(left_speed, 1023), 0)
        right_speed = max(min(right_speed, 1023), 0)

        velocities = {}
        for dxl_id in LEFT_MOTORS:
            velocities[dxl_id] = left_speed
        for dxl_id in RIGHT_MOTORS:
            velocities[dxl_id] = right_speed

        self.set_velocities_bulk(velocities)

    def shutdown(self):
        try:
            self.stop_all()
            for dxl_id in DXL_IDS:
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0
                )
            self.portHandler.closePort()
            print("Motors stopped and port closed")
        except Exception as e:
            print(f"Motor shutdown error: {e}")