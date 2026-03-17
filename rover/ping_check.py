import os
from dynamixel_sdk import *

# --- SETTINGS ---
DEVICE = '/dev/ttyUSB0'  # Check with 'ls /dev/ttyUSB*'
BAUD = 57600             # Try 1000000 if 57600 fails
PROTOCOL = 2.0           # Use 1.0 for older AX/RX servos

port = PortHandler(DEVICE)
packet = PacketHandler(PROTOCOL)

# Open Port
if not port.openPort():
    print("Failed to open port. Try: sudo chmod 666 /dev/ttyUSB0")
    exit()
port.setBaudRate(BAUD)

# 1. Try Ping ID 1
print(f"Pinging ID 1 at {BAUD} bps...")
model, res, err = packet.ping(port, 1)

if res == COMM_SUCCESS:
    print(f"SUCCESS! Found ID 1 (Model Number: {model})")
else:
    # 2. Backup: Scan IDs 0-10
    print("ID 1 not found. Scanning IDs 0-10...")
    for i in range(11):
        _, res, _ = packet.ping(port, i)
        if res == COMM_SUCCESS:
            print(f"FOUND! Your servo is actually ID: {i}")
            break
    else:
        print("No servos detected. Check 12V power and baud rate.")

port.closePort()
