import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from tf2_ros import Buffer, TransformException, TransformListener


class InteractiveSetpoint(Node):
    """RViz goals in map frame (matches RTAB-Map + drone_config.rviz Fixed Frame: drone/map)."""

    def __init__(self):
        super().__init__('interactive_setpoint')

        self.map_frame = self.declare_parameter('map_frame', 'drone/map').value
        self.base_frame = self.declare_parameter('base_frame', 'drone/drone_link').value
        self.marker_z_offset_m = float(self.declare_parameter('marker_z_offset_m', 2.0).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.server = InteractiveMarkerServer(self, 'drone_setpoint')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_3d', 10)

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.map_frame
        self.int_marker.name = '3d_goal'
        self.int_marker.description = 'Drone 3D Target'
        self.int_marker.scale = 1.0

        tx_control = InteractiveMarkerControl()
        tx_control.name = "move_x"
        tx_control.orientation.w = 1.0
        tx_control.orientation.x = 1.0
        tx_control.orientation.y = 0.0
        tx_control.orientation.z = 0.0
        tx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(tx_control)

        ty_control = InteractiveMarkerControl()
        ty_control.name = "move_y"
        ty_control.orientation.w = 1.0
        ty_control.orientation.x = 0.0
        ty_control.orientation.y = 1.0
        ty_control.orientation.z = 0.0
        ty_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(ty_control)

        tz_control = InteractiveMarkerControl()
        tz_control.name = "move_z"
        tz_control.orientation.w = 1.0
        tz_control.orientation.x = 0.0
        tz_control.orientation.y = 0.0
        tz_control.orientation.z = 1.0
        tz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(tz_control)

        rz_control = InteractiveMarkerControl()
        rz_control.name = "rotate_z"
        rz_control.orientation.w = 1.0
        rz_control.orientation.x = 0.0
        rz_control.orientation.y = 1.0
        rz_control.orientation.z = 0.0
        rz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(rz_control)

        free_control = InteractiveMarkerControl()
        free_control.name = "move_free"
        free_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        free_control.always_visible = True

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

        self.initialized_position = False
        self.create_timer(0.4, self._try_init_marker_from_tf)

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Send Goal to Drone", callback=self.menu_callback)

        self.get_logger().info(
            f"Interactive setpoint: waiting for TF {self.map_frame} <- {self.base_frame}..."
        )

    def _try_init_marker_from_tf(self):
        if self.initialized_position:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.3)
            )
        except TransformException:
            return

        self.int_marker.pose.position.x = t.transform.translation.x
        self.int_marker.pose.position.y = t.transform.translation.y
        self.int_marker.pose.position.z = t.transform.translation.z + self.marker_z_offset_m
        self.int_marker.pose.orientation = t.transform.rotation

        self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
        self.menu_handler.apply(self.server, self.int_marker.name)
        self.server.applyChanges()
        self.initialized_position = True
        self.get_logger().info(
            f"Marker snapped in {self.map_frame} near drone. RViz Fixed Frame should be "
            f"{self.map_frame}."
        )

    def process_feedback(self, feedback):
        pass

    def menu_callback(self, feedback):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.pose = feedback.pose

        self.goal_pub.publish(goal_pose)

        self.get_logger().info(
            f"Published goal in {self.map_frame}: "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}, "
            f"{goal_pose.pose.position.z:.2f})"
        )


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
