import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from octomap_msgs.msg import Octomap
import math

# A very basic stub for the 3D Planner since A* on an octomap without python bindings is complex
# It simply forwards the goal pose to the cmd_pose topic for Phase 2, but provides the structure
# for Phase 3 Octomap integration later.

class DronePlanner(Node):
    def __init__(self):
        super().__init__('drone_planner')

        self.cmd_pub = self.create_publisher(PoseStamped, '/cmd_pose', 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose_3d', self.goal_cb, 10)
            
        # self.octomap_sub = self.create_subscription(
        #     Octomap, '/rtabmap/octomap_grid', self.octomap_cb, 10) # Example topic

        self.get_logger().info("Drone Planner initialized (Direct Waypoint Mode).")
        self.current_octomap = None

    def octomap_cb(self, msg):
        # Save octomap for planning
        self.current_octomap = msg

    def goal_cb(self, msg):
        # In a full implementation, you would run A* here between the current pose (from TF)
        # and the msg pose, checking self.current_octomap for collisions.
        
        self.get_logger().info("Received new goal. Forwarding direct to drone (No obstacle avoidance yet).")
        
        # For now, just pass the goal directly to the drone driver (Phase 2 behavior)
        self.cmd_pub.publish(msg)

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
