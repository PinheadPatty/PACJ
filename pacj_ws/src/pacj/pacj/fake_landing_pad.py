import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import LandingTargetPose

class FakeLandingPad(Node):
    def __init__(self):
        super().__init__('fake_landing_pad')

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher to PX4
        self.landing_target_pub = self.create_publisher(
            LandingTargetPose, '/fmu/in/landing_target_pose', qos_profile)

        # Subscriber to virtual rover position (from teleop or another script)
        self.rover_pose_sub = self.create_subscription(
            PoseStamped, '/rover_pose', self.rover_pose_cb, 10)

        # Current Rover Position in ROS ENU (defaults to origin if no rover is running)
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_z = 0.0

        # Publish at 10Hz to keep Precision Landing mode happy
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("Fake Landing Pad tracking /rover_pose initialized.")

    def rover_pose_cb(self, msg):
        # Update the rover's position when we receive a new command
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y
        self.rover_z = msg.pose.position.z

    def timer_cb(self):
        msg = LandingTargetPose()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # The rover is moving, so the target is NOT static
        msg.is_static = False 

        # We will provide absolute coordinates in the Local NED frame
        msg.abs_pos_valid = True
        
        # Convert ROS ENU to PX4 NED
        # ENU X -> NED Y (East)
        # ENU Y -> NED X (North)
        # ENU Z -> NED Z (Down - inverted)
        msg.x_abs = float(self.rover_y)
        msg.y_abs = float(self.rover_x)
        msg.z_abs = -float(self.rover_z)

        # We are not providing relative position/velocity right now (just absolute)
        msg.rel_pos_valid = False
        msg.rel_vel_valid = False

        self.landing_target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLandingPad()
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
