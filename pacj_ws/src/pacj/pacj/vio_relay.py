import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleVisualOdometry

class VioRelay(Node):
    def __init__(self):
        super().__init__('vio_relay')

        # Configure QoS profile for RX and TX (PX4 DDS uses Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher to PX4
        self.vio_pub = self.create_publisher(
            VehicleVisualOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)

        # Subscriber from RTAB-Map
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10) # default RTAB-map odom topic

        # Add a timeout so we don't spam the console
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info("VIO Relay initialized.")

    def euler_from_quaternion(self, w, x, y, z):
        # roll (x)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # yaw (z)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        return [w, x, y, z]

    def odom_cb(self, msg):
        vio_msg = VehicleVisualOdometry()
        
        vio_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vio_msg.timestamp_sample = vio_msg.timestamp
        
        # ROS 2 geometry_msgs uses ENU (East, North, Up)
        # PX4 uses NED (North, East, Down)
        
        ros_x = float(msg.pose.pose.position.x)
        ros_y = float(msg.pose.pose.position.y)
        ros_z = float(msg.pose.pose.position.z)
        
        # Convert ENU to NED
        vio_msg.x = ros_y   # North = Y
        vio_msg.y = ros_x   # East = X
        vio_msg.z = -ros_z  # Down = -Z
        
        # Log occasionally to prove we are receiving RTAB-Map data
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 2e9:  # Log every 2 seconds
            self.get_logger().info(f"Relaying RTAB-Map Odom to PX4 -> X: {ros_x:.2f}, Y: {ros_y:.2f}, Z: {ros_z:.2f}")
            self.last_log_time = now
            
        # Extract ENU Euler angles
        q = msg.pose.pose.orientation
        roll_enu, pitch_enu, yaw_enu = self.euler_from_quaternion(q.w, q.x, q.y, q.z)
        
        # Convert ENU to NED
        # In ENU: East is 0, North is pi/2
        # In NED: North is 0, East is pi/2
        roll_ned = roll_enu
        pitch_ned = -pitch_enu
        yaw_ned = (math.pi / 2.0) - yaw_enu
        
        # Convert back to quaternion for PX4 (NED)
        q_ned = self.quaternion_from_euler(roll_ned, pitch_ned, yaw_ned)
        
        vio_msg.q[0] = q_ned[0] # w
        vio_msg.q[1] = q_ned[1] # x
        vio_msg.q[2] = q_ned[2] # y
        vio_msg.q[3] = q_ned[3] # z

        # Velocity ENU to NED
        ros_vx = float(msg.twist.twist.linear.x)
        ros_vy = float(msg.twist.twist.linear.y)
        ros_vz = float(msg.twist.twist.linear.z)
        
        vio_msg.vx = ros_vy
        vio_msg.vy = ros_vx
        vio_msg.vz = -ros_vz

        # Angular Velocity ENU to NED
        ros_wx = float(msg.twist.twist.angular.x)
        ros_wy = float(msg.twist.twist.angular.y)
        ros_wz = float(msg.twist.twist.angular.z)
        
        vio_msg.rollspeed = ros_wy
        vio_msg.pitchspeed = ros_wx
        vio_msg.yawspeed = -ros_wz

        # Velocity frame is body frame
        vio_msg.velocity_frame = VehicleVisualOdometry.VELOCITY_FRAME_BODY_FRD

        # Coordinate frames
        vio_msg.local_frame = VehicleVisualOdometry.LOCAL_FRAME_NED

        self.vio_pub.publish(vio_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VioRelay()
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
