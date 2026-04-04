import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class DroneDriver(Node):
    def __init__(self):
        super().__init__('drone_driver')

        # Configure QoS profile for RX and TX (PX4 DDS uses Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers to PX4
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers from PX4
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)

        # Subscribers from Teleop (VM)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        
        self.start_time = self.get_clock().now()
        self.arm_req_time = self.get_clock().now()
        self.mode_req_time = self.get_clock().now()

        # PX4 Offboard requires continuous streaming of setpoints at >2Hz
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info("Micro XRCE-DDS Drone Driver Initialized.")

    def vehicle_status_cb(self, msg):
        # Keep track of the drone's actual state
        if self.nav_state != msg.nav_state:
            self.get_logger().info(f"Nav state changed to: {msg.nav_state}")
        if self.arming_state != msg.arming_state:
            self.get_logger().info(f"Arming state changed to: {msg.arming_state}")
            
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmd_vel_cb(self, msg):
        self.current_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_cb(self):
        now = self.get_clock().now()
        
        # NOTE: Removed 0.5s safety timeout because teleop_twist_keyboard 
        # only sends messages on key presses, not continuously.
        # In a real flight with a joystick, you should restore a timeout here!
        # if (now - self.last_cmd_time).nanoseconds > 5e8:
        #     self.current_twist = Twist()

        # We must publish these constantly to keep PX4 happy in OFFBOARD mode
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Auto-arm and switch to OFFBOARD mode 
        # Handshake: Only try to arm/switch mode after 1 second of streaming setpoints
        if (now - self.start_time).nanoseconds > 1e9:
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if (now - self.mode_req_time).nanoseconds > 1e9:  # Limit requests to 1Hz
                    # Command PX4 to switch to Offboard mode
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # 1.0=Custom mode, 6.0=OFFBOARD
                    self.mode_req_time = now
                    
            elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                if (now - self.arm_req_time).nanoseconds > 1e9:  # Limit requests to 1Hz
                    # Command PX4 to arm
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # 1.0=Arm
                    self.arm_req_time = now

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        # Coordinate Frame Translation:
        # ROS 2 (cmd_vel) uses FLU (Forward, Left, Up)
        # PX4 uses NED (North, East, Down)
        msg.velocity[0] = self.current_twist.linear.x   # North = Forward
        msg.velocity[1] = -self.current_twist.linear.y  # East = -Left (Right)
        msg.velocity[2] = -self.current_twist.linear.z  # Down = -Up
        
        # Yaw rate (Z axis rotation). 
        # ROS is counter-clockwise positive (Up), PX4 is clockwise positive (Down)
        msg.yawspeed = -self.current_twist.angular.z

        # In velocity control mode, position and acceleration must be explicitly set to NaN
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
