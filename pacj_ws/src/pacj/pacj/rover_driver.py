import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver')

        # --- XL430-W250-T Control Table ---
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE  = 64
        self.ADDR_VELOCITY_LIMIT = 44   
        self.ADDR_GOAL_VELOCITY  = 104  
        self.VELOCITY_MODE       = 1
        self.BAUDRATE            = 57600
        self.DEVICE_NAME         = '/dev/ttyUSB0' 
        self.IDS                 = [1, 2] # Left=1, Right=2

        # --- Settings for "Slow" Movement ---
        self.SPEED_LIMIT = 50   # Physical cap (~11 RPM)
        self.WHEEL_SEP   = 0.14 # Meters between wheels
        self.SCALE       = 100.0 # Sensitivity (Keyboard input * 40)

        # --- SDK Initialization ---
        self.port_handler = PortHandler(self.DEVICE_NAME)
        self.packet_handler = PacketHandler(2.0)
        self.groupSyncWrite = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_VELOCITY, 4)
        
        if not self.port_handler.openPort():
            self.get_logger().error("Could not open port! Try: sudo chmod 666 /dev/ttyUSB0")
            return
        self.port_handler.setBaudRate(self.BAUDRATE)

        # --- One-Time Hardware Setup ---
        for dxl_id in self.IDS:
            print(f"Configuring Motor ID: {dxl_id}")
            
            # 1. Disable torque to allow setting changes
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
            
            # 2. UNLOCK: Set Velocity Limit
            # This fixes the "stiff but not spinning" issue
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_VELOCITY_LIMIT, 200)
            
            # 3. Force Velocity Mode (1)
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, self.VELOCITY_MODE)
            
            # 4. Re-enable Torque (This makes them stiff/ready)
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)


        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Rover Driver Online. Use teleop_twist_keyboard to drive.")

    def cmd_vel_callback(self, msg):
        # Differential Drive Math
        left_vel  = (msg.linear.x - (msg.angular.z * self.WHEEL_SEP / 2.0)) * self.SCALE
        right_vel = (msg.linear.x + (msg.angular.z * self.WHEEL_SEP / 2.0)) * self.SCALE
        print(left_vel, right_vel)

        # 1. Clear any data from the previous command
        self.groupSyncWrite.clearParam()

        # 2. Convert to integers and then to 4-byte little-endian format
        # Note: Using -right_vel for the mirrored mounting
        param_left  = [DXL_LOBYTE(DXL_LOWORD(int(left_vel))), DXL_HIBYTE(DXL_LOWORD(int(left_vel))), 
                    DXL_LOBYTE(DXL_HIWORD(int(left_vel))), DXL_HIBYTE(DXL_HIWORD(int(left_vel)))]
                    
        param_right = [DXL_LOBYTE(DXL_LOWORD(int(-right_vel))), DXL_HIBYTE(DXL_LOWORD(int(-right_vel))), 
                    DXL_LOBYTE(DXL_HIWORD(int(-right_vel))), DXL_HIBYTE(DXL_HIWORD(int(-right_vel)))]

        # 3. Add motors to the sync group
        self.groupSyncWrite.addParam(1, param_left)
        self.groupSyncWrite.addParam(2, param_right)

        # 4. Send the single "Sync" packet to the bus
        self.groupSyncWrite.txPacket()

def main():
    rclpy.init()
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Stop and loosen motors on exit
        for dxl_id in [1, 2]:
            node.packet_handler.write4ByteTxOnly(node.port_handler, dxl_id, 104, 0)
            node.packet_handler.write1ByteTxRx(node.port_handler, dxl_id, 64, 0)
        node.destroy_node()
        rclpy.shutdown()
