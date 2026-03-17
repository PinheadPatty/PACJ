import sys
from dynamixel_sdk import *

# --- SETTINGS ---
DEVICE = '/dev/ttyUSB0'
BAUD = 57600        # Try 1000000 if 57600 fails
PROTOCOL = 2.0      # Use 1.0 for AX/RX servos
ADDR_ID = 7         # Control Table Address for ID (Protocol 2.0)

# 1. Check for command line argument
if len(sys.argv) < 2:
    print("Usage: python3 set_id.py <NEW_ID_INTEGER>")
    sys.exit()

try:
    NEW_ID = int(sys.argv[1])
except ValueError:
    print("Error: ID must be an integer.")
    sys.exit()

port = PortHandler(DEVICE)
packet = PacketHandler(PROTOCOL)

if not port.openPort() or not port.setBaudRate(BAUD):
    print("Port Error. Try: sudo chmod 666 /dev/ttyUSB0")
    sys.exit()

# 2. Use Broadcast ID (254) to change the ID
print(f"Attempting to change connected servo to ID {NEW_ID}...")
result, error = packet.write1ByteTxRx(port, 254, ADDR_ID, NEW_ID)

if result == COMM_SUCCESS:
    print(f"SUCCESS! The servo is now ID {NEW_ID}.")
    print("IMPORTANT: Power cycle the servo (unplug/replug) to save changes.")
else:
    print(f"FAILED: {packet.getTxRxResult(result)}")
    print("Check 12V power and baud rate (try 1000000).")

port.closePort()
