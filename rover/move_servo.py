import os
from dynamixel_sdk import * # Uses Dynamixel SDK library

# --- Setup Parameters ---
MY_ID          = 3               # Check your Motor ID
BAUDRATE       = 57600           # Check your Baudrate
DEVICENAME     = '/dev/ttyUSB0'  # Check your Port
PROTOCOL_VERSION = 2.0

# Addresses (X-Series)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE  = 64
ADDR_GOAL_VELOCITY  = 104
VELOCITY_MODE       = 1
TORQUE_ENABLE       = 1
TARGET_SPEED        = 50        # Adjust this (e.g., 50-200 for slow test)

# Initialize Handlers
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open Port & Set Baudrate
if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Port Open Success!")

    # 1. Set Operating Mode to Velocity (Torque must be OFF to change mode)
    packetHandler.write1ByteTxRx(portHandler, MY_ID, ADDR_OPERATING_MODE, VELOCITY_MODE)
    
    # 2. Enable Torque
    packetHandler.write1ByteTxRx(portHandler, MY_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    # 3. Set Goal Velocity (Spinning starts now!)
    print(f"Spinning motor {MY_ID} at speed {TARGET_SPEED}...")
    packetHandler.write4ByteTxRx(portHandler, MY_ID, ADDR_GOAL_VELOCITY, TARGET_SPEED)

    # Wait for user to stop
    input("Press Enter to stop the motor...")

    # 4. Stop and Disable
    packetHandler.write4ByteTxRx(portHandler, MY_ID, ADDR_GOAL_VELOCITY, 0)
    packetHandler.write1ByteTxRx(portHandler, MY_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("Finished.")
else:
    print("Failed to open port.")
