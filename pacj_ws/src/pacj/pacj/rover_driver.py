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
        self.SCALE       = 40.0 # Sensitivity (Keyboard input * 40)

        # --- SDK Initialization ---
        self.port_handler = PortHandler(self.DEVICE_NAME)
        self.packet_handler = PacketHandler(2.0)
        
        if not self.port_handler.openPort():
            self.get_logger().error("Could not open port! Try: sudo chmod 666 /dev/ttyUSB0")
            return
        self.port_handler.setBaudRate(self.BAUDRATE)

        # --- One-Time Hardware Setup ---
        for dxl_id in self.IDS:
            # Must disable torque to change EEPROM settings (Mode/Limits)
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_VELOCITY_LIMIT, self.SPEED_LIMIT)
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, self.VELOCITY_MODE)
            # Re-enable Torque
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Rover Driver Online. Use teleop_twist_keyboard to drive.")

    def cmd_vel_callback(self, msg):
        # Differential Drive Math
        left_vel  = (msg.linear.x - (msg.angular.z * self.WHEEL_SEP / 2.0)) * self.SCALE
        right_vel = (msg.linear.x + (msg.angular.z * self.WHEEL_SEP / 2.0)) * self.SCALE

        # Send Commands (Inverting right motor 2 for mirrored mounting)
        self.packet_handler.write4ByteTxOnly(self.port_handler, 1, self.ADDR_GOAL_VELOCITY, int(left_vel))
        self.packet_handler.write4ByteTxOnly(self.port_handler, 2, self.ADDR_GOAL_VELOCITY, int(-right_vel))

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
