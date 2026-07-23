import time
from dynamixel_sdk import *

DXL_DEVICENAME = '/dev/ttyUSB0'
DXL_PROTOCOL_VERSION = 1.0
DXL_BAUDRATE = 57600
DXL_IDS = [2, 7, 8, 9]
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_VELOCITY = 32
ROTATE_SPEED = 300
TURN_DURATION = 2.0

portHandler = PortHandler(DXL_DEVICENAME)
packetHandler = PacketHandler(DXL_PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_VELOCITY, 2)

if not portHandler.openPort() or not portHandler.setBaudRate(DXL_BAUDRATE):
    print("Failed to open DXL port")
    quit()

for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1)

print("Motors initialized")

def set_velocities(velocities_dict):
    groupSyncWrite.clearParam()
    for dxl_id, velocity in velocities_dict.items():
        if dxl_id in [7, 8]:
            velocity = -velocity
        velocity = max(min(velocity, 1023), -1023)
        if velocity < 0:
            velocity = 1024 - velocity
        param = [DXL_LOBYTE(velocity), DXL_HIBYTE(velocity)]
        groupSyncWrite.addParam(dxl_id, param)
    groupSyncWrite.txPacket()

def stop_all():
    set_velocities({dxl_id: 0 for dxl_id in DXL_IDS})

def rotate_left(speed):
    set_velocities({2: -speed, 9: -speed, 7: speed, 8: speed})

def rotate_right(speed):
    set_velocities({2: speed, 9: speed, 7: -speed, 8: -speed})

try:
    print("Rotating LEFT for", TURN_DURATION, "seconds")
    rotate_left(ROTATE_SPEED)
    time.sleep(TURN_DURATION)
    stop_all()
    time.sleep(0.5)

    print("Rotating RIGHT for", TURN_DURATION, "seconds")
    rotate_right(ROTATE_SPEED)
    time.sleep(TURN_DURATION)
    stop_all()

    print("Test complete")

except KeyboardInterrupt:
    print("Interrupted")
finally:
    stop_all()
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("Motors stopped and port closed")