import threading
import time

import rclpy
from dynamixel_sdk import *
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

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
        self.POS_OPEN     = None
        self.CLOSE_OFFSET = int(self.declare_parameter('coupler_close_offset', 1000).value)
        self.POS_CLOSED   = None
        self.COUPLE_SPEED = 30      # Set once at startup (Steady crawl)
        self.SAFE_PWM     = 250     # Power cap to protect gears
        self.COUPLE_TOLERANCE = 30
        
        # --- 2. CONTROL TABLE ADDRESSES ---
        self.ADDR_OPERATING_MODE  = 11
        self.ADDR_PWM_LIMIT       = 36
        self.ADDR_VELOCITY_LIMIT  = 44
        self.ADDR_TORQUE_ENABLE   = 64
        self.ADDR_PROF_VELOCITY   = 112
        self.ADDR_GOAL_POSITION   = 116
        self.ADDR_GOAL_VELOCITY   = 104
        self.ADDR_PRESENT_POSITION = 132
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
        self.capture_coupler_open()
        if self.POS_OPEN is None:
            self.get_logger().warn(
                "Coupler open position not initialized. Restart with coupler open."
            )

        # --- 4. ROS INTERFACES ---
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cpl_sub = self.create_subscription(String, 'coupling', self.coupling_callback, 10)
        self.cpl_status_pub = self.create_publisher(String, 'coupling_status', 10)
        self.create_timer(1.0, self.publish_coupler_status)
        self.get_logger().info("--- Rover Driver Online ---")
        self.get_logger().info(
            "Coupler commands on /coupling: '0' (open), '1' (close), "
            "'open', 'close', any value in [0,1], 'relax', 'status'"
        )
        self.get_logger().info(
            f"Current coupler endpoints: open={self.POS_OPEN}, "
            f"close={self.POS_CLOSED} (offset={self.CLOSE_OFFSET})"
        )

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
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_OPERATING_MODE, 4) # Position Mode
            self.packet_handler.write2ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_PWM_LIMIT, self.SAFE_PWM)
            # Set the speed limit ONCE while torque is off
            self.packet_handler.write4ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_PROF_VELOCITY, self.COUPLE_SPEED)
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 1)
            # Error Code 0x20 is Overload. If triggered, the LED will blink and torque will drop.
            self.packet_handler.write1ByteTxRx(self.port_handler, self.COUPLE_ID, 48, 0x20)

    def recompute_closed_position(self):
        self.POS_CLOSED = self.POS_OPEN - self.CLOSE_OFFSET

    def read_coupler_position(self):
        with self.comm_lock:
            position, res, err = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.COUPLE_ID, self.ADDR_PRESENT_POSITION
            )

        if res != COMM_SUCCESS:
            self.get_logger().error(
                f"Read position failed: {self.packet_handler.getTxRxResult(res)}"
            )
            return None
        if err != 0:
            self.get_logger().error(
                f"Read position hardware error: {self.packet_handler.getRxPacketError(err)}"
            )
            return None

        return int(position)

    def set_torque(self, enabled):
        with self.comm_lock:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.COUPLE_ID, self.ADDR_TORQUE_ENABLE, 1 if enabled else 0
            )

    def capture_coupler_open(self):
        position = self.read_coupler_position()
        if position is None:
            return

        self.POS_OPEN = position
        self.recompute_closed_position()
        self.get_logger().info(
            f"Captured OPEN endpoint at {position}; CLOSE auto-set to {self.POS_CLOSED} "
            f"(offset={self.CLOSE_OFFSET})"
        )

    def get_target_position_from_command(self, command):
        if self.POS_OPEN is None or self.POS_CLOSED is None:
            self.get_logger().error(
                "Coupler position not initialized. Restart with coupler open."
            )
            return None, None

        if self.POS_OPEN == self.POS_CLOSED:
            self.get_logger().error(
                "Open and close endpoints are identical. Recalibrate with "
                "'relax' then 'set_open'."
            )
            return None, None

        if command == 'open':
            normalized = 0.0
        elif command == 'close':
            normalized = 1.0
        else:
            try:
                normalized = float(command)
            except ValueError:
                return None, None

        if normalized < 0.0 or normalized > 1.0:
            self.get_logger().warn(
                f"Coupler value {normalized:.3f} is outside [0,1], clamping."
            )
        normalized = max(0.0, min(1.0, normalized))

        target = int(round(self.POS_OPEN + normalized * (self.POS_CLOSED - self.POS_OPEN)))
        return target, normalized

    def move_coupler(self, target_pos, normalized):
        self.set_torque(True)
        with self.comm_lock:
            res, err = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                self.COUPLE_ID,
                self.ADDR_GOAL_POSITION,
                target_pos,
            )

        if res != COMM_SUCCESS:
            self.get_logger().error(f"Coupler move failed: {self.packet_handler.getTxRxResult(res)}")
        elif err != 0:
            self.get_logger().error(f"Coupler hardware error: {self.packet_handler.getRxPacketError(err)}")
        else:
            self.get_logger().info(
                f"Coupler moving to normalized={normalized:.2f} -> position {target_pos}"
            )

    def relax_coupler_for_calibration(self, duration_sec=30):
        self.set_torque(False)
        self.get_logger().info(
            "Calibration mode: torque OFF for 30s. Move coupler by hand, then restart "
            "rover_driver with coupler open."
        )

        for i in range(duration_sec):
            position = self.read_coupler_position()
            if position is not None:
                self.get_logger().info(f"Calibration step {i+1}/{duration_sec} - Position: {position}")
            time.sleep(1.0)

        self.get_logger().info("Calibration window ended. Send motion command to re-enable torque.")

    def get_coupler_status(self):
        position = self.read_coupler_position()
        if position is None or self.POS_OPEN is None or self.POS_CLOSED is None:
            return "unknown"

        if abs(position - self.POS_OPEN) <= self.COUPLE_TOLERANCE:
            return "uncoupled"
        if abs(position - self.POS_CLOSED) <= self.COUPLE_TOLERANCE:
            return "coupled"
        return "moving"

    def publish_coupler_status(self):
        if self.cpl_status_pub.get_subscription_count() == 0:
            return
        status = self.get_coupler_status()
        msg = String()
        msg.data = status
        self.cpl_status_pub.publish(msg)

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

    # TO RELAX:
    # ros2 topic pub --once /coupling std_msgs/String "data: 'relax'"

    # TO OPEN:
    # ros2 topic pub --once /coupling std_msgs/String "data: '0'"

    # TO CLOSE:
    # ros2 topic pub --once /coupling std_msgs/String "data: '1'"
    
    
    def coupling_callback(self, msg):
        command = msg.data.lower().strip()

        if command == 'relax':
            self.relax_coupler_for_calibration()
            return

        if command == 'set_close':
            self.get_logger().warn(
                "set_close is deprecated. Close is auto-computed as open + offset. "
                "Use 'set_open' only."
            )
            return

        if command == 'status':
            self.get_logger().info(
                f"Coupler endpoints: open={self.POS_OPEN}, close={self.POS_CLOSED}, "
                f"offset={self.CLOSE_OFFSET}"
            )
            return

        target_pos, normalized = self.get_target_position_from_command(command)
        if target_pos is None:
            self.get_logger().warn(
                f"Unknown coupling command: '{command}'. Use 0/1, open/close, "
                "or a value in [0,1]."
            )
            return

        self.move_coupler(target_pos, normalized)
        self.publish_coupler_status()
   
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
