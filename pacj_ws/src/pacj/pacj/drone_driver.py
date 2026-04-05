import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleAttitude

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
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_cb, qos_profile)

        # Subscribers from Teleop (VM)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.cmd_pose_sub = self.create_subscription(
            PoseStamped, '/cmd_pose', self.cmd_pose_cb, 10)

        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_yaw = 0.0
        
        self.control_mode = 'VELOCITY'  # Toggles between 'VELOCITY' and 'POSITION'
        self.current_twist = Twist()
        self.current_pose = PoseStamped()
        
        # Default target pose (so it doesn't fly into the ground if you switch early)
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 2.0  # 2 meters up
        
        self.last_cmd_time = self.get_clock().now()
        
        self.start_time = self.get_clock().now()
        self.arm_req_time = self.get_clock().now()
        self.mode_req_time = self.get_clock().now()

        # PX4 Offboard requires continuous streaming of setpoints at >2Hz
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info("Micro XRCE-DDS Drone Driver Initialized.")

    def vehicle_attitude_cb(self, msg):
        q = msg.q
        self.current_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

    def vehicle_status_cb(self, msg):
        # Keep track of the drone's actual state
        if self.nav_state != msg.nav_state:
            self.get_logger().info(f"Nav state changed to: {msg.nav_state}")
        if self.arming_state != msg.arming_state:
            self.get_logger().info(f"Arming state changed to: {msg.arming_state}")
            
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmd_vel_cb(self, msg):
        self.control_mode = 'VELOCITY'
        self.current_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def cmd_pose_cb(self, msg):
        self.control_mode = 'POSITION'
        self.current_pose = msg
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

        # NOTE: Automatic arming and offboard switching has been removed.
        # You must now arm and switch to Offboard mode manually using your joystick/QGC!

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        if self.control_mode == 'POSITION':
            msg.position = True
            msg.velocity = False
        else:
            msg.position = False
            msg.velocity = True
            
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        if self.control_mode == 'VELOCITY':
            # 1. Convert FLU to Body-NED (Forward, Right, Down)
            xy_multiplier = 5.0
            z_multiplier = 1.0
            
            body_forward = float(self.current_twist.linear.x) * xy_multiplier
            body_right = -float(self.current_twist.linear.y) * xy_multiplier
            body_down = -float(self.current_twist.linear.z) * z_multiplier
            
            # 2. Rotate Body-NED to Local-NED using current yaw
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            
            msg.velocity[0] = body_forward * cos_yaw - body_right * sin_yaw  # North
            msg.velocity[1] = body_forward * sin_yaw + body_right * cos_yaw  # East
            msg.velocity[2] = body_down                                      # Down
            
            # Yaw rate (Z axis rotation). 
            msg.yawspeed = -float(self.current_twist.angular.z)

            # In velocity control mode, position and acceleration must be explicitly set to NaN
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.jerk = [float('nan'), float('nan'), float('nan')]
            msg.yaw = float('nan')
            
            # LOGGING
            if abs(msg.velocity[0]) > 0.05 or abs(msg.velocity[1]) > 0.05 or abs(msg.velocity[2]) > 0.05 or abs(msg.yawspeed) > 0.05:
                self.get_logger().info(f"Sending PX4 Vel -> N: {msg.velocity[0]:.2f}, E: {msg.velocity[1]:.2f}, D: {msg.velocity[2]:.2f}")

        elif self.control_mode == 'POSITION':
            # ROS 2 geometry_msgs uses ENU (East, North, Up)
            # PX4 uses NED (North, East, Down)
            
            ros_x = float(self.current_pose.pose.position.x)
            ros_y = float(self.current_pose.pose.position.y)
            ros_z = float(self.current_pose.pose.position.z)
            
            # Convert ENU to NED
            msg.position[0] = ros_y   # North = Y
            msg.position[1] = ros_x   # East = X
            msg.position[2] = -ros_z  # Down = -Z
            
            # We don't control yaw in this simple example, just maintain current heading
            msg.yaw = float(self.current_yaw)
            
            # Ignore velocity and acceleration
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.jerk = [float('nan'), float('nan'), float('nan')]
            msg.yawspeed = float('nan')
            
            self.get_logger().info(f"Sending PX4 Pos -> N: {msg.position[0]:.2f}, E: {msg.position[1]:.2f}, D: {msg.position[2]:.2f}")

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
        msg.source_component = 191  # 191 = MAV_COMP_ID_ONBOARD_COMPUTER (Required so PX4 doesn't ignore it!)
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
