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
            
        # Convert quaternion from ENU to NED
        # 1. ENU to NED for positions: [x, y, z] -> [y, x, -z]
        # 2. For Quaternions [x, y, z, w], mapping is: 
        # q_ned_x = q_enu_y
        # q_ned_y = q_enu_x
        # q_ned_z = -q_enu_z
        # q_ned_w = q_enu_w
        
        q_enu_x = msg.pose.pose.orientation.x
        q_enu_y = msg.pose.pose.orientation.y
        q_enu_z = msg.pose.pose.orientation.z
        q_enu_w = msg.pose.pose.orientation.w
        
        vio_msg.q[0] = q_enu_w
        vio_msg.q[1] = q_enu_y
        vio_msg.q[2] = q_enu_x
        vio_msg.q[3] = -q_enu_z

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
