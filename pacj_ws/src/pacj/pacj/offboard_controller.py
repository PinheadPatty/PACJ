#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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
    - While PX4 is in OFFBOARD mode, any message on /drone/cmd_pose immediately
      engages TRACK_CMD. No separate arming step is required.
    - Each OFFBOARD edge (enter or exit) returns to HOLD_LOCAL so a stale
      streamed /drone/cmd_pose cannot resume after mode changes.
    - cmd_timeout_s: if no /drone/cmd_pose arrives for this many seconds the
      controller falls back to HOLD_LOCAL. This is the only safety net against
      a dead planner. Set to 0 to disable.

    Clamping policy:
    - No position or yaw clamping is applied here. The planner is responsible
      for commanding safe lookahead targets. This node is intentionally kept
      as a thin, low-latency bridge so direct emergency overrides on
      /drone/cmd_pose always take effect immediately.
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
        self.mode_pub = self.create_publisher(String, '/drone/offboard_controller/mode', 10)

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
            '/drone/fmu/out/vehicle_status_v1',
            self.vehicle_status_cb,
            qos_profile,
        )
        self.create_subscription(PoseStamped, '/drone/cmd_pose', self.cmd_pose_cb, 10)

        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 2.0).value)

        self.current_yaw = 0.0
        self.drone_pos = [0.0, 0.0, 0.0]
        self.mode = 'HOLD_LOCAL'
        self.last_published_mode = None
        self.cmd_target = PoseStamped()
        self.offboard_active = False
        self.vehicle_status_received = False
        self.last_cmd_time_s = -1.0

        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 2.0
        self.current_pose.pose.orientation.w = 1.0

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self._publish_mode_if_changed()
        self.get_logger().info(
            'Offboard controller ready. In OFFBOARD mode, stream /drone/cmd_pose to track.'
        )

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def _publish_mode_if_changed(self):
        if self.mode == self.last_published_mode:
            return
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)
        self.last_published_mode = self.mode

    def _enter_hold_local(self, reason: str = ''):
        previously_tracking = self.mode == 'TRACK_CMD'
        self.mode = 'HOLD_LOCAL'
        self._latch_current_hold_target()
        self.last_published_mode = None  # force republish even on HOLD->HOLD
        if previously_tracking or reason:
            self.get_logger().info(f'HOLD_LOCAL{": " + reason if reason else ""}')

    def _flush_on_offboard_edge(self):
        self._enter_hold_local(reason='offboard edge')

    def _can_track(self):
        return self.mode == 'TRACK_CMD' and self.offboard_active

    # ------------------------------------------------------------------
    # Pose helpers
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

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
            self._flush_on_offboard_edge()
            self.get_logger().info(
                'PX4 entered OFFBOARD: stream /drone/cmd_pose to track.'
            )

        if was_offboard and (not self.offboard_active):
            had_track = self.mode == 'TRACK_CMD'
            self._flush_on_offboard_edge()
            if had_track:
                self.get_logger().info('PX4 exited OFFBOARD while tracking.')

    def cmd_pose_cb(self, msg):
        """Accept the pose directly with no clamping. The planner owns lookahead limiting."""
        if not self.offboard_active:
            return

        self.cmd_target = msg
        self.mode = 'TRACK_CMD'
        self.last_cmd_time_s = float(self.get_clock().now().nanoseconds) * 1e-9

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def timer_cb(self):
        if self.mode == 'TRACK_CMD' and self.cmd_timeout_s > 0.0 and self.last_cmd_time_s > 0.0:
            now_s = float(self.get_clock().now().nanoseconds) * 1e-9
            if (now_s - self.last_cmd_time_s) > self.cmd_timeout_s:
                self._enter_hold_local(
                    reason=f'no /drone/cmd_pose for {self.cmd_timeout_s:.1f}s (planner dead?)'
                )

        if self.mode == 'TRACK_CMD' and self.vehicle_status_received and (not self.offboard_active):
            self._enter_hold_local(reason='PX4 not in OFFBOARD')

        if self._can_track():
            self.current_pose = self.cmd_target
        else:
            self._latch_current_hold_target()

        self._publish_mode_if_changed()
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    # ------------------------------------------------------------------
    # PX4 publishers
    # ------------------------------------------------------------------

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