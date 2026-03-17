import time
from dynamixel_sdk import * # Uses Dynamixel SDK library

# --- Configuration ---
ADDR_PRESENT_VOLTAGE = 144   
PROTOCOL_VERSION     = 2.0
DXL_ID               = 1     
BAUDRATE             = 57600 
DEVICENAME           = '/dev/ttyUSB0' 

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port and set baudrate
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("Failed to open port or set baudrate.")
    quit()

print("🔋 3S Battery Monitor Active! Press Ctrl+C to stop.\n")

try:
    while True:
        # Read the voltage from the servo
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VOLTAGE)

        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            actual_voltage = dxl_present_voltage / 10.0
            
            # --- 3S LiPo Percentage Math ---
            max_v = 12.6 # 100% Charge
            min_v = 9.6  # 0% Charge (Absolute lowest safe limit)
            
            # Calculate percentage
            percentage = ((actual_voltage - min_v) / (max_v - min_v)) * 100
            
            # Clamp the percentage between 0% and 100%
            percentage = max(0.0, min(100.0, percentage))
            
            # Print the stats formatted nicely
            print(f"Voltage: {actual_voltage:.1f}V  |  Charge: {percentage:.0f}%")
            
            # Trigger an alert if the battery gets to 15% or lower
            if percentage <= 15:
                print("🚨 WARNING: BATTERY CRITICALLY LOW! CHARGE SOON! 🚨")
                
        else:
            print("Failed to read from DYNAMIXEL. Check connection.")
            
        # Wait 2 seconds before checking again so we don't spam the terminal
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopping battery monitor...")

# Clean up and close the port
portHandler.closePort()
