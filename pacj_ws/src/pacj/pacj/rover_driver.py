import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dynamixel_sdk import *
import threading

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver')

        # --- 1. USER CONFIGURATION ---
        self.DEVICE_NAME = '/dev/ttyUSB0'
        self.BAUDRATE    = 57600
        
        # Drive Settings (IDs 1 & 2)
        self.DRIVE_IDS    = [1, 2]
        self.WHEEL_SEP    = 0.13
        self.DRIVE_SCALE  = 125.0
        self.TURN_GAIN    = 8.0
        self.DRIVE_LIMIT  = 200
        
        # Coupling Settings (ID 3)
        self.COUPLE_ID    = 3
        self.POS_OPEN     = 2570     
        self.POS_CLOSED   = 3500    
        self.COUPLE_SPEED = 30      # Set once at startup (Steady crawl)
        self.SAFE_PWM     = 250     # Power cap to protect gears
        
        # --- 2. CONTROL TABLE ADDRESSES ---
        self.ADDR_OPERATING_MODE  = 11
        self.ADDR_PWM_LIMIT       = 36
        self.ADDR_VELOCITY_LIMIT  = 44
        self.ADDR_TORQUE_ENABLE   = 64
        self.ADDR_PROF_VELOCITY   = 112
        self.ADDR_GOAL_POSITION   = 116
        self.ADDR_GOAL_VELOCITY   = 104
        self.ADDR_PRESENT_VOLTAGE = 144

        # --- 3. HARDWARE INIT ---
        self.comm_lock = threading.Lock()
        self.port_handler = PortHandler(self.DEVICE_NAME)
        self.packet_handler = PacketHandler(2.0)
        self.groupSyncWrite = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_VELOCITY, 4)
        
        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Serial Link Failed! Check U2D2 connection.")
            return

        self.setup_hardware()

        # --- 4. ROS INTERFACES ---
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cpl_sub = self.create_subscription(String, 'coupling', self.coupling_callback, 10)
        self.batt_pub = self.create_publisher(String, 'battery_status', 10)
        
        # Smart Battery Timer (Polls only if someone is watching)
        self.create_timer(2.0, self.battery_monitor_callback)

        self.get_logger().info("--- Rover Driver Online ---")
        self.get_logger().info("Send 'open' or 'close' to /coupling")

    def setup_hardware(self):
        """Initializes all motors. Torque is toggled here to set limits safely."""
        with self.comm_lock:
            # Drive Motors Initialization
            for dxl_id in self.DRIVE_IDS:
                self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
                self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_VELOCITY_LIMIT, self.DRIVE_LIMIT)
                self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, 1)
                self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)
            
            # Coupling Motor Initialization
            self.get_logger().info(f"Configuring Coupling Motor (ID {self.COUPLE_ID})...")
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 0)
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_OPERATING_MODE, 3) # Position Mode
            self.packet_handler.write2ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_PWM_LIMIT, self.SAFE_PWM)
            # Set the speed limit ONCE while torque is off
            self.packet_handler.write4ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_PROF_VELOCITY, self.COUPLE_SPEED)
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 1)
            # Error Code 0x20 is Overload. If triggered, the LED will blink and torque will drop.
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, 48, 0x20)

    def get_4byte_param(self, value):
        val = int(value)
        return [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)), 
                DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]

    # TO DRIVE: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    def cmd_vel_callback(self, msg):
        left  = (msg.linear.x - (msg.angular.z * self.WHEEL_SEP / 2.0) * self.TURN_GAIN) * self.DRIVE_SCALE
        right = (msg.linear.x + (msg.angular.z * self.WHEEL_SEP / 2.0) * self.TURN_GAIN) * self.DRIVE_SCALE
        with self.comm_lock:
            self.groupSyncWrite.clearParam()
            self.groupSyncWrite.addParam(1, self.get_4byte_param(left))
            self.groupSyncWrite.addParam(2, self.get_4byte_param(-right))
            self.groupSyncWrite.txPacket()

    # TO OPEN: ros2 topic pub --once /coupling std_msgs/String "data: 'open'"
    # TO CLOSE: ros2 topic pub --once /coupling std_msgs/String "data: 'close'"
    # TO RELAX:  ros2 topic pub --once /coupling std_msgs/String "data: 'relax'"
    def coupling_callback(self, msg):
        import time
        command = msg.data.lower().strip()

        # --- 1. CALIBRATION MODE ('relax') ---
        # This unlocks the motor and prints the position for 30 seconds
        if command == 'relax':
            with self.comm_lock:
                self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 0)
            
            self.get_logger().info("!!! CALIBRATION START: Motor Unlocked for 30s !!!")
            
            for i in range(30):
                with self.comm_lock:
                    pos, res, err = self.packet_handler.read4ByteTxRx(
                        self.port_handler, self.COUPLE_ID, 132 # Present Position
                    )
                if res == 0: # COMM_SUCCESS
                    self.get_logger().info(f"Step {i+1}/30 - Position: {pos}")
                else:
                    self.get_logger().error("Read Failed! Check connection.")
                time.sleep(1.0) # Poll once per second
                
            self.get_logger().info("!!! CALIBRATION ENDED. Send 'open' or 'close' to re-engage. !!!")
            return

        # --- 2. MOVEMENT COMMANDS ('open' / 'close') ---
        target_pos = self.POS_OPEN if command == 'open' else self.POS_CLOSED if command == 'close' else None

        if target_pos is None:
            self.get_logger().warn(f"Unknown command: {command}")
            return

        with self.comm_lock:
            # SAFETY: Ensure torque is ON before moving (re-engages after relax)
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 1)
            
            # Send the goal position
            res, err = self.packet_handler.write4ByteTxRx(
                self.port_handler, 
                self.COUPLE_ID, 
                self.ADDR_GOAL_POSITION, 
                target_pos
            )
            
            if res != 0: # Not COMM_SUCCESS
                self.get_logger().error(f"Coupler Move FAILED: {self.packet_handler.getTxRxResult(res)}")
            elif err != 0:
                self.get_logger().error(f"Coupler Hardware Error: {self.packet_handler.getRxPacketError(err)}")
            else:
                self.get_logger().info(f"Coupler moving to {command.upper()} ({target_pos})")
   
    # TO CHECK: ros2 topic echo /battery_status --once
    def battery_monitor_callback(self):
        if self.batt_pub.get_subscription_count() > 0:
            with self.comm_lock:
                volt_raw, res, err = self.packet_handler.read2ByteTxRx(self.port_handler, 1, self.ADDR_PRESENT_VOLTAGE)
            if res == COMM_SUCCESS and err == 0:
                v = volt_raw / 10.0
                pct = max(0.0, min(100.0, ((v - 9.6) / (12.6 - 9.6)) * 100))
                msg = String()
                msg.data = f"Battery: {v:.1f}V ({pct:.0f}%)"
                self.batt_pub.publish(msg)

def main():
    rclpy.init()
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for dxl_id in [1, 2, 3]:
            node.packet_handler.write1ByteTxRx(node.port_handler, dxl_id, 64, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()