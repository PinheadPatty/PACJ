import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
        self.int_marker.scale = 1.0

        # Create a more dynamic, full 6-DOF control setup
        # This will add the familiar RViz rings (for rotation/yaw) and arrows (for translation)
        
        # --- TRANSLATION CONTROLS (Arrows) ---
        
        # X-axis Translation (Red Arrow)
        tx_control = InteractiveMarkerControl()
        tx_control.name = "move_x"
        tx_control.orientation.w = 1.0
        tx_control.orientation.x = 1.0
        tx_control.orientation.y = 0.0
        tx_control.orientation.z = 0.0
        tx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(tx_control)
        
        # Y-axis Translation (Green Arrow)
        ty_control = InteractiveMarkerControl()
        ty_control.name = "move_y"
        ty_control.orientation.w = 1.0
        ty_control.orientation.x = 0.0
        ty_control.orientation.y = 1.0
        ty_control.orientation.z = 0.0
        ty_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(ty_control)
        
        # Z-axis Translation (Blue Arrow)
        tz_control = InteractiveMarkerControl()
        tz_control.name = "move_z"
        tz_control.orientation.w = 1.0
        tz_control.orientation.x = 0.0
        tz_control.orientation.y = 0.0
        tz_control.orientation.z = 1.0
        tz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(tz_control)
        
        # --- ROTATION CONTROLS (Rings) ---
        
        # Z-axis Rotation / Yaw (Blue Ring)
        # This is the most important one for drones!
        rz_control = InteractiveMarkerControl()
        rz_control.name = "rotate_z"
        rz_control.orientation.w = 1.0
        rz_control.orientation.x = 0.0
        rz_control.orientation.y = 1.0
        rz_control.orientation.z = 0.0
        rz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(rz_control)

        # --- CENTER DRAG CONTROL (Free movement) ---
        # A box in the middle to drag it around freely
        free_control = InteractiveMarkerControl()
        free_control.name = "move_free"
        free_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        free_control.always_visible = True
        
        # Add a visual box to the free control so you know where to grab it
        from visualization_msgs.msg import Marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.25
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.25
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.8
        free_control.markers.append(box_marker)
        
        self.int_marker.controls.append(free_control)

        # Subscribe to PX4 odometry so we can snap the marker to the drone on startup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.initialized_position = False

        self.get_logger().info("Interactive setpoint waiting for drone position to initialize...")

    def odom_cb(self, msg):
        if not self.initialized_position:
            # Snap to drone's current position, but 2 meters up
            self.int_marker.pose.position.x = float(msg.position[1])  # East
            self.int_marker.pose.position.y = float(msg.position[0])  # North
            self.int_marker.pose.position.z = -float(msg.position[2]) + 2.0 # Up + 2m
            
            self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
            self.server.applyChanges()
            self.initialized_position = True
            
            self.get_logger().info(f"Snapped marker to drone -> X: {self.int_marker.pose.position.x:.2f}, Y: {self.int_marker.pose.position.y:.2f}. Ready to move in RViz!")

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