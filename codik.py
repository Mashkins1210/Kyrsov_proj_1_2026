import os
import time
from dynamixel_sdk import *

# Connection parameters
DEVICENAME                  = '/dev/ttyUSB0'
PROTOCOL_VERSION            = 1.0
BAUDRATE                    = 57600

# Register addresses (protocol 1.0)
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_GOAL_VELOCITY       = 32

# Motor IDs 
DXL_IDs = [2, 7, 8, 9]  

# Initialize port
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set baud rate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable torque for all motors
for dxl_id in DXL_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Dynamixel {dxl_id} torque enabled")

# Function to set velocity
def set_velocity(dxl_id, velocity):
    # Limit velocity (-1023 to 1023)
    velocity = max(min(velocity, 1023), -1023)
    
    # Convert negative velocity for protocol
    if velocity < 0:
        velocity = 1024 - velocity
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, dxl_id, ADDR_MX_GOAL_VELOCITY, velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"DXL {dxl_id} velocity set to {velocity}")

def move_all_forward(duration):
    print("Moving all wheels forward")
    for dxl_id in DXL_IDs:
        set_velocity(dxl_id, 500)  # all motors forward
    time.sleep(duration)          # move forward for specified duration
    stop_all()                   # Stop all motors after duration

def stop_all():
    print("Stopping all wheels")
    for dxl_id in DXL_IDs:
        set_velocity(dxl_id, 0)   # Stop

try:
    move_all_forward(5) #motors 3 seconds
except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # Disable torque for all motors
    for dxl_id in DXL_IDs:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0)
        print(f"Dynamixel {dxl_id} torque disabled")
    
    portHandler.closePort()
    print("Port closed")
