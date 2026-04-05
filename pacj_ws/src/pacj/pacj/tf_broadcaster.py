import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
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

        self.get_logger().info("TF Broadcaster initialized.")

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
        
        # Convert NED to ENU
        ros_x = px4_e   # East = X
        ros_y = px4_n   # North = Y
        ros_z = -px4_d  # Up = -Down
        
        t.transform.translation.x = ros_x
        t.transform.translation.y = ros_y
        t.transform.translation.z = ros_z

        # Extract Quaternion from message and convert NED to ENU
        # msg.q is in order [w, x, y, z] for px4
        q_ned_w = float(msg.q[0])
        q_ned_x = float(msg.q[1])
        q_ned_y = float(msg.q[2])
        q_ned_z = float(msg.q[3])
        
        q_enu_x = q_ned_y
        q_enu_y = q_ned_x
        q_enu_z = -q_ned_z
        q_enu_w = q_ned_w

        t.transform.rotation.x = q_enu_x
        t.transform.rotation.y = q_enu_y
        t.transform.rotation.z = q_enu_z
        t.transform.rotation.w = q_enu_w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

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
