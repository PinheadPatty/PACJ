import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # --- Configuration ---
        self.ADDR_PRESENT_VOLTAGE = 144
        self.PROTOCOL_VERSION     = 2.0
        self.DXL_ID               = 1
        self.BAUDRATE             = 57600
        self.DEVICENAME           = '/dev/ttyUSB0'
        
        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to open port or set baudrate.")
            return

        self.get_logger().info("3S Battery Monitor Active!")
        
        # Publisher
        self.publisher_ = self.create_publisher(BatteryState, '/rover/battery', 10)
        
        # Timer (run every 2 seconds)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
    def timer_callback(self):
        dxl_present_voltage, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VOLTAGE)
            
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            actual_voltage = dxl_present_voltage / 10.0
            
            # --- 3S LiPo Percentage Math ---
            max_v = 12.6
            min_v = 9.6
            
            percentage = ((actual_voltage - min_v) / (max_v - min_v)) * 100
            percentage = max(0.0, min(100.0, percentage))
            
            msg = BatteryState()
            msg.voltage = actual_voltage
            msg.percentage = percentage / 100.0 # Standard ROS BatteryState uses 0 to 1
            msg.present = True
            
            self.publisher_.publish(msg)
            
            self.get_logger().info(f'Published Battery: {actual_voltage:.1f}V | {percentage:.0f}%')
            
            if percentage <= 15:
                self.get_logger().warn("🚨 WARNING: BATTERY CRITICALLY LOW! CHARGE SOON! 🚨")
        else:
            self.get_logger().error("Failed to read from DYNAMIXEL. Check connection.")

    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryNode()
    try:
        rclpy.spin(battery_node)
    except KeyboardInterrupt:
        pass
    finally:
        battery_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
