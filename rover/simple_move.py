import sys, tty, termios
from dynamixel_sdk import *

# --- CONFIG ---
ID_RIGHT = 1
ID_LEFT  = 2
BAUD     = 57600       # Try 1000000 if this fails
ADDR_VEL = 104         # Velocity Address for X-Series (Protocol 2.0)
LEN_VEL  = 4
DEVICE   = '/dev/ttyUSB0'

# Setup Port
port = PortHandler(DEVICE)
packet = PacketHandler(2.0)

if not port.openPort() or not port.setBaudRate(BAUD):
    print("Failed to open port. Try: sudo chmod 666 /dev/ttyUSB0")
    sys.exit()

# Function to set velocity
def set_vel(id, vel):
    packet.write4ByteTxRx(port, id, ADDR_VEL, vel)

# Keyboard Input Setup
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("Controls: W (Forward), S (Back), A (Left), D (Right), Space (Stop), Q (Quit)")

try:
    # Enable Torque (Address 64 for X-series)
    packet.write1ByteTxRx(port, ID_LEFT, 64, 1)
    packet.write1ByteTxRx(port, ID_RIGHT, 64, 1)

    while True:
        key = getch().lower()
        if key == 'w': set_vel(ID_LEFT, 100);  set_vel(ID_RIGHT, -100)
        elif key == 's': set_vel(ID_LEFT, -100); set_vel(ID_RIGHT, 100)
        elif key == 'a': set_vel(ID_LEFT, -50);  set_vel(ID_RIGHT, -50)
        elif key == 'd': set_vel(ID_LEFT, 50);   set_vel(ID_RIGHT, 50)
        elif key == ' ': set_vel(ID_LEFT, 0);    set_vel(ID_RIGHT, 0)
        elif key == 'q': break
finally:
    set_vel(ID_LEFT, 0); set_vel(ID_RIGHT, 0)
    port.closePort()
