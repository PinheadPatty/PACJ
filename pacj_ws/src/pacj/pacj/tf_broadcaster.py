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
        # PX4 q is [w, x, y, z] representing rotation from NED to FRD
        # We need the rotation from ENU to FLU.
        a = 0.7071068
        w_ned = float(msg.q[0])
        x_ned = float(msg.q[1])
        y_ned = float(msg.q[2])
        z_ned = float(msg.q[3])
        
        # A simpler, much more reliable conversion from NED (PX4) to ENU (ROS)
        # 1. Yaw needs to be rotated 90 degrees CCW (North -> Y axis, East -> X axis)
        # 2. Pitch and Roll axes need to be flipped
        q_enu_w = w_ned
        q_enu_x = y_ned
        q_enu_y = x_ned
        q_enu_z = -z_ned
        
        # In ROS, the marker natively points along the X axis.
        # In our translation logic, we set X to East and Y to North.
        # Since the marker arrow defaults to pointing along X (East), we need to rotate 
        # the quaternion by 90 degrees around Z to make the arrow point North (Y).
        
        q_offset_w = 0.7071068
        q_offset_z = 0.7071068
        
        final_w = q_enu_w * q_offset_w - q_enu_z * q_offset_z
        final_x = q_enu_x * q_offset_w + q_enu_y * q_offset_z
        final_y = q_enu_y * q_offset_w - q_enu_x * q_offset_z
        final_final_z = q_enu_w * q_offset_z + q_enu_z * q_offset_w

        t.transform.rotation.x = final_x
        t.transform.rotation.y = final_y
        t.transform.rotation.z = final_final_z
        t.transform.rotation.w = final_w

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
        
        # Rotate the marker 90 degrees around Z axis (yaw) so the arrow points North (Y axis) instead of East (X axis)
        # Quaternion for 90 degree Z rotation: w=0.707, x=0, y=0, z=0.707
        # We need to multiply the drone's orientation by this 90 degree offset
        
        q_drone = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        q_offset = [0.0, 0.0, 0.7071068, 0.7071068]
        
        # Quaternion multiplication: q_result = q_drone * q_offset
        marker.pose.orientation.w = q_drone[3]*q_offset[3] - q_drone[0]*q_offset[0] - q_drone[1]*q_offset[1] - q_drone[2]*q_offset[2]
        marker.pose.orientation.x = q_drone[3]*q_offset[0] + q_drone[0]*q_offset[3] + q_drone[1]*q_offset[2] - q_drone[2]*q_offset[1]
        marker.pose.orientation.y = q_drone[3]*q_offset[1] - q_drone[0]*q_offset[2] + q_drone[1]*q_offset[3] + q_drone[2]*q_offset[0]
        marker.pose.orientation.z = q_drone[3]*q_offset[2] + q_drone[0]*q_offset[1] - q_drone[1]*q_offset[0] + q_drone[2]*q_offset[3]
        
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
