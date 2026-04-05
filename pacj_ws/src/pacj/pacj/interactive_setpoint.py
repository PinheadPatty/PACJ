import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class InteractiveSetpoint(Node):
    def __init__(self):
        super().__init__('interactive_setpoint')

        self.server = InteractiveMarkerServer(self, 'drone_setpoint')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_3d', 10)

        # Create the interactive marker
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = 'odom'
        self.int_marker.name = '3d_goal'
        self.int_marker.description = 'Drone 3D Target'
        
        # Initial position (e.g. at 2m height)
        self.int_marker.pose.position.x = 0.0
        self.int_marker.pose.position.y = 0.0
        self.int_marker.pose.position.z = 2.0
        self.int_marker.scale = 1.0

        # Add movement controls (X, Y, Z translation)
        
        # X-axis control
        control_x = InteractiveMarkerControl()
        control_x.orientation.w = 1.0
        control_x.orientation.x = 1.0
        control_x.orientation.y = 0.0
        control_x.orientation.z = 0.0
        control_x.name = "move_x"
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control_x)
        
        # Y-axis control
        control_y = InteractiveMarkerControl()
        control_y.orientation.w = 1.0
        control_y.orientation.x = 0.0
        control_y.orientation.y = 1.0
        control_y.orientation.z = 0.0
        control_y.name = "move_y"
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control_y)
        
        # Z-axis control
        control_z = InteractiveMarkerControl()
        control_z.orientation.w = 1.0
        control_z.orientation.x = 0.0
        control_z.orientation.y = 0.0
        control_z.orientation.z = 1.0
        control_z.name = "move_z"
        control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control_z)
        
        # XY-plane control (drag around on a flat plane)
        control_xy = InteractiveMarkerControl()
        control_xy.orientation.w = 1.0
        control_xy.orientation.x = 0.0
        control_xy.orientation.y = 1.0
        control_xy.orientation.z = 0.0
        control_xy.name = "move_xy"
        control_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.int_marker.controls.append(control_xy)
        
        # Free 3D movement (click and drag the center box anywhere)
        control_free = InteractiveMarkerControl()
        control_free.name = "move_free"
        control_free.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control_free.always_visible = True
        self.int_marker.controls.append(control_free)

        self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()
        
        self.get_logger().info("Interactive setpoint initialized. You can move the marker in RViz.")

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'odom'
            goal_pose.pose = feedback.pose
            
            self.goal_pub.publish(goal_pose)
            
            self.get_logger().info(f"Published new 3D goal -> X: {goal_pose.pose.position.x:.2f}, Y: {goal_pose.pose.position.y:.2f}, Z: {goal_pose.pose.position.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveSetpoint()
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