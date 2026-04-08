import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
import math

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

class VioRelay(Node):
    def __init__(self):
        super().__init__('vio_relay')

        # 1. Force Simulation Time
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # 2. PX4 DDS Agent compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vio_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)

        # Ensure this matches your RTAB-Map output
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom', self.odom_cb, 10)

        self.last_log_time = self.get_clock().now()
        self.get_logger().info("VIO Relay [Audited] initialized. Mapping ENU -> NED...")

    def odom_cb(self, msg):
        try:
            vio_msg = VehicleOdometry()
            
            # Timestamp in microseconds (SITL requirement)
            current_time = int(self.get_clock().now().nanoseconds / 1000)
            vio_msg.timestamp = current_time
            vio_msg.timestamp_sample = current_time
            
            # 1. POSITION CONVERSION (ENU -> NED)
            # ROS X (Forward/East) -> PX4 X (North)
            # ROS Y (Left/North)   -> PX4 Y (East)
            # ROS Z (Up)           -> PX4 Z (Down)
            vio_msg.position = [
                float(msg.pose.pose.position.x),
                float(-msg.pose.pose.position.y),
                float(-msg.pose.pose.position.z)
            ]
            
            # 2. ORIENTATION CONVERSION (Corrected ENU -> NED)
            # Mapping components to align World-North and World-East
            q = msg.pose.pose.orientation
            vio_msg.q = [
                float(q.w), 
                float(q.y),  # Map North
                float(q.x),  # Map East
                float(-q.z)  # Invert Up to Down
            ]

            # 3. VELOCITY (Set to NaN for smoother initial testing)
            # This forces PX4 to derive velocity from position, stopping the 'twitch'
            vio_msg.velocity = [float('nan')] * 3
            vio_msg.angular_velocity = [float('nan')] * 3

            # 4. Frame Definitions & Trust
            vio_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            vio_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            
            # High trust values (low variance)
            vio_msg.position_variance = [0.1, 0.1, 0.1]
            vio_msg.orientation_variance = [0.05, 0.05, 0.05]
            
            vio_msg.reset_counter = 0
            
            self.vio_pub.publish(vio_msg)

            # Log every 2 seconds
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds > 2e9:
                self.get_logger().info(f"Relaying VIO @ {vio_msg.position[0]:.2f}, {vio_msg.position[1]:.2f}")
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Error in odom_cb: {str(e)}")

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
