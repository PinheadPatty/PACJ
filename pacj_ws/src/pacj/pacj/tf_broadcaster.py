import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        # Configure QoS profile for RX and TX (PX4 DDS uses Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber from PX4
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Marker Publisher (to easily see the drone in RViz)
        self.marker_pub = self.create_publisher(Marker, '/drone_marker', 10)

        self.get_logger().info("TF Broadcaster initialized.")

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
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # PX4 uses NED (North, East, Down)
        # ROS 2 geometry_msgs uses ENU (East, North, Up)
        
        px4_n = float(msg.position[0])
        px4_e = float(msg.position[1])
        px4_d = float(msg.position[2])
        
        # Convert NED to ENU properly for translation
        # PX4 uses NED (North, East, Down)
        # ROS 2 geometry_msgs uses ENU (East, North, Up)
        
        # When PX4 moves forward (North), it should move up the Y-axis in ROS.
        # When PX4 moves right (East), it should move up the X-axis in ROS.
        
        ros_x = px4_e   # East = X
        ros_y = px4_n   # North = Y
        ros_z = -px4_d  # Up = -Down
        
        t.transform.translation.x = ros_x
        t.transform.translation.y = ros_y
        t.transform.translation.z = ros_z

        # Extract Quaternion from message and convert NED to ENU properly
        # We use explicit Euler angle conversion to prevent any quaternion handedness issues
        roll_ned, pitch_ned, yaw_ned = self.euler_from_quaternion(
            float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
        )
        
        # Convert NED Euler to ENU Euler
        # NED (North-East-Down) to ENU (East-North-Up)
        roll_enu = roll_ned
        pitch_enu = -pitch_ned
        yaw_enu = (math.pi / 2.0) - yaw_ned
        
        # Convert back to quaternion for ROS (FLU)
        q_enu = self.quaternion_from_euler(roll_enu, pitch_enu, yaw_enu)
        
        t.transform.rotation.w = q_enu[0]
        t.transform.rotation.x = q_enu[1]
        t.transform.rotation.y = q_enu[2]
        t.transform.rotation.z = q_enu[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        # Publish a visual marker for the drone
        marker = Marker()
        marker.header.stamp = t.header.stamp
        marker.header.frame_id = 'odom'
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position the arrow at the drone's position in the odom frame
        marker.pose.position.x = t.transform.translation.x
        marker.pose.position.y = t.transform.translation.y
        marker.pose.position.z = t.transform.translation.z
        
        # Orient the arrow to match the drone's orientation in the odom frame
        marker.pose.orientation.x = t.transform.rotation.x
        marker.pose.orientation.y = t.transform.rotation.y
        marker.pose.orientation.z = t.transform.rotation.z
        marker.pose.orientation.w = t.transform.rotation.w
        
        # Size of the arrow
        marker.scale.x = 1.0  # Length
        marker.scale.y = 0.2  # Width
        marker.scale.z = 0.2  # Height
        
        # Color (Yellow, opaque)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
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
