#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardController(Node):
    """Position-only PX4 offboard bridge.

    Always publishes PX4 position setpoints with a local-position hold baseline.

    Tracking policy:
    - /cmd_pose is ignored unless PX4 is in OFFBOARD and tracking is armed
      (subscribe to /offboard_controller/tracking_armed, std_msgs/Bool).
    - Each OFFBOARD edge (enter or exit) clears arming and returns to HOLD_LOCAL
      so a stale streamed /cmd_pose cannot resume after mode changes.
    - After arming, /cmd_pose engages TRACK_CMD; disarm returns to HOLD_LOCAL
      at the current pose.
    """

    def __init__(self):
        super().__init__('offboard_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/drone/fmu/in/offboard_control_mode',
            qos_profile,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/drone/fmu/in/trajectory_setpoint',
            qos_profile,
        )
        self.mode_pub = self.create_publisher(String, '/offboard_controller/mode', 10)

        self.create_subscription(
            VehicleAttitude,
            '/drone/fmu/out/vehicle_attitude',
            self.vehicle_attitude_cb,
            qos_profile,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/drone/fmu/out/vehicle_local_position',
            self.vehicle_local_pos_cb,
            qos_profile,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            qos_profile,
        )

        self.create_subscription(PoseStamped, '/cmd_pose', self.cmd_pose_cb, 10)
        self.create_subscription(Bool, '/offboard_controller/tracking_armed', self.tracking_armed_cb, 10)

        self.max_position_step_m = float(self.declare_parameter('max_position_step_m', 0.5).value)
        self.max_yaw_step_rad = float(self.declare_parameter('max_yaw_step_rad', 0.35).value)
        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 2.0).value)

        self.current_yaw = 0.0
        self.drone_pos = [0.0, 0.0, 0.0]
        self.mode = 'HOLD_LOCAL'
        self.last_published_mode = None
        self.cmd_target = PoseStamped()
        self.offboard_active = False
        self.vehicle_status_received = False
        self.last_cmd_time_s = -1.0

        self.tracking_armed = False

        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 2.0
        self.current_pose.pose.orientation.w = 1.0

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self._publish_mode_if_changed()
        self.get_logger().info(
            'Offboard controller: hold by default. In OFFBOARD, publish '
            'true to /offboard_controller/tracking_armed then /cmd_pose to track.'
        )

    def _publish_mode_if_changed(self):
        if self.mode == self.last_published_mode:
            return
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)
        self.last_published_mode = self.mode

    def _flush_tracking_on_offboard_edge(self):
        """Disarm and hold whenever OFFBOARD state enters or exits (stale goal guard)."""
        self.tracking_armed = False
        self.mode = 'HOLD_LOCAL'
        self._latch_current_hold_target()

    def _set_current_pose_from_ned(self, x_north, y_east, z_down, yaw_ned):
        self.current_pose.pose.position.x = float(y_east)
        self.current_pose.pose.position.y = float(x_north)
        self.current_pose.pose.position.z = float(-z_down)

        ros_yaw = -float(yaw_ned)
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = math.sin(ros_yaw / 2.0)
        self.current_pose.pose.orientation.w = math.cos(ros_yaw / 2.0)

    def _latch_current_hold_target(self):
        self._set_current_pose_from_ned(
            self.drone_pos[0],
            self.drone_pos[1],
            self.drone_pos[2],
            self.current_yaw,
        )

    def _yaw_from_orientation(self, q_x, q_y, q_z, q_w):
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_pi(angle_rad):
        while angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2.0 * math.pi
        return angle_rad

    def _ros_yaw_from_ned_yaw(self, yaw_ned):
        return self._wrap_pi(-float(yaw_ned))

    def vehicle_attitude_cb(self, msg):
        q = msg.q
        self.current_yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]),
        )

    def vehicle_local_pos_cb(self, msg):
        self.drone_pos = [msg.x, msg.y, msg.z]

    def vehicle_status_cb(self, msg):
        was_offboard = self.offboard_active
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.vehicle_status_received = True

        if (not was_offboard) and self.offboard_active:
            self._flush_tracking_on_offboard_edge()
            self.get_logger().info(
                'PX4 entered OFFBOARD: disarmed tracking; publish '
                'true to /offboard_controller/tracking_armed then /cmd_pose to track.'
            )

        if was_offboard and (not self.offboard_active):
            had_track = self.mode == 'TRACK_CMD'
            self._flush_tracking_on_offboard_edge()
            if had_track:
                self.get_logger().info('PX4 exited OFFBOARD; returning to HOLD_LOCAL.')

    def tracking_armed_cb(self, msg):
        was_armed = self.tracking_armed
        self.tracking_armed = bool(msg.data)
        if self.tracking_armed and (not was_armed):
            self.get_logger().info('Tracking armed; publish /cmd_pose (while OFFBOARD) to engage TRACK_CMD.')
        if (not self.tracking_armed) and was_armed:
            if self.mode == 'TRACK_CMD':
                self.mode = 'HOLD_LOCAL'
                self._latch_current_hold_target()
                self.get_logger().info('Tracking disarmed; HOLD_LOCAL.')

    def cmd_pose_cb(self, msg):
        if not self.offboard_active:
            return
        if not self.tracking_armed:
            return

        anchor_x = float(self.drone_pos[1])
        anchor_y = float(self.drone_pos[0])
        anchor_z = float(-self.drone_pos[2])

        req_x = float(msg.pose.position.x)
        req_y = float(msg.pose.position.y)
        req_z = float(msg.pose.position.z)

        dx = req_x - anchor_x
        dy = req_y - anchor_y
        dz = req_z - anchor_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance > self.max_position_step_m and distance > 1e-6:
            scale = self.max_position_step_m / distance
            req_x = anchor_x + dx * scale
            req_y = anchor_y + dy * scale
            req_z = anchor_z + dz * scale
            self.get_logger().warn(
                f'/cmd_pose clamped to {self.max_position_step_m:.2f} m from current position.'
            )

        msg.pose.position.x = req_x
        msg.pose.position.y = req_y
        msg.pose.position.z = req_z

        cmd_yaw = self._yaw_from_orientation(
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        cur_ros_yaw = self._ros_yaw_from_ned_yaw(self.current_yaw)
        dyaw = self._wrap_pi(cmd_yaw - cur_ros_yaw)
        if abs(dyaw) > self.max_yaw_step_rad:
            dyaw = math.copysign(self.max_yaw_step_rad, dyaw)
            self.get_logger().warn(
                f'/cmd_pose yaw delta clamped to {self.max_yaw_step_rad:.3f} rad.'
            )
        yaw = self._wrap_pi(cur_ros_yaw + dyaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.cmd_target = msg
        self.mode = 'TRACK_CMD'
        self.last_cmd_time_s = float(self.get_clock().now().nanoseconds) * 1e-9

    def timer_cb(self):
        if self.mode == 'TRACK_CMD' and (not self.tracking_armed):
            self.mode = 'HOLD_LOCAL'
            self._latch_current_hold_target()

        if self.mode == 'TRACK_CMD' and self.cmd_timeout_s > 0.0 and self.last_cmd_time_s > 0.0:
            now_s = float(self.get_clock().now().nanoseconds) * 1e-9
            if (now_s - self.last_cmd_time_s) > self.cmd_timeout_s:
                self.mode = 'HOLD_LOCAL'
                self._latch_current_hold_target()
                self.get_logger().warn('No /cmd_pose update within cmd_timeout_s; returning to HOLD_LOCAL.')

        if self.mode == 'TRACK_CMD' and self.vehicle_status_received and (not self.offboard_active):
            self.mode = 'HOLD_LOCAL'
            self._latch_current_hold_target()

        if self.mode == 'TRACK_CMD' and self.offboard_active and self.tracking_armed:
            self.current_pose = self.cmd_target
        else:
            self._latch_current_hold_target()

        self._publish_mode_if_changed()
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        ros_x = float(self.current_pose.pose.position.x)
        ros_y = float(self.current_pose.pose.position.y)
        ros_z = float(self.current_pose.pose.position.z)
        msg.position[0] = ros_y
        msg.position[1] = ros_x
        msg.position[2] = -ros_z

        q_x = self.current_pose.pose.orientation.x
        q_y = self.current_pose.pose.orientation.y
        q_z = self.current_pose.pose.orientation.z
        q_w = self.current_pose.pose.orientation.w
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        target_yaw = math.atan2(siny_cosp, cosy_cosp)
        msg.yaw = -target_yaw

        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
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
