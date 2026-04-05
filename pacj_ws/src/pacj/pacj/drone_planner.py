import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

# A very basic stub for the 3D Planner since A* on an octomap without python bindings is complex
# It simply forwards the goal pose to the cmd_pose topic for Phase 2, but provides the structure
# for Phase 3 Octomap integration later.

class DronePlanner(Node):
    def __init__(self):
        super().__init__('drone_planner')

        self.cmd_pub = self.create_publisher(PoseStamped, '/cmd_pose', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose_3d', self.goal_cb, 10)
            
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)

        self.get_logger().info("Drone Planner initialized (Direct Waypoint Mode).")
        self.current_octomap = None
        self.current_pose = PoseStamped()

    def odom_cb(self, msg):
        # Keep track of the drone's current position to draw the path
        # Convert PX4 NED to ROS 2 ENU
        self.current_pose.pose.position.x = float(msg.position[1])  # East = X
        self.current_pose.pose.position.y = float(msg.position[0])  # North = Y
        self.current_pose.pose.position.z = -float(msg.position[2]) # Up = -Down

    def octomap_cb(self, msg):
        # Save octomap for planning
        self.current_octomap = msg

    def goal_cb(self, msg):
        # In a full implementation, you would run A* here between the current pose (from TF)
        # and the msg pose, checking self.current_octomap for collisions.
        
        self.get_logger().info("Received new goal. Forwarding direct to drone (No obstacle avoidance yet).")
        
        # For now, just pass the goal directly to the drone driver (Phase 2 behavior)
        self.cmd_pub.publish(msg)
        
        # Publish a simple straight-line Path for visualization in RViz
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose.position = self.current_pose.pose.position
        
        end_pose = PoseStamped()
        end_pose.header = path.header
        end_pose.pose.position = msg.pose.position
        
        path.poses = [start_pose, end_pose]
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = DronePlanner()
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
