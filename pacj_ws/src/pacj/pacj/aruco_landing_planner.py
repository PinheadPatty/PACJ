#!/usr/bin/env python3
"""
ArUco landing helper: one slow ramp toward the vision goal (X, Y, Z, yaw together).

Goal updates each fresh /drone/aruco/relative_pose (FMU + body XY, marker range for Z, ArUco yaw).
Publishes /drone/cmd_pose in the same frame as offboard_controller._set_current_pose_from_ned.

  ros2 run pacj aruco_landing_planner
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def quat_normalize(q: Quaternion) -> Quaternion:
    n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) or 1.0
    out = Quaternion()
    out.w = q.w / n
    out.x = q.x / n
    out.y = q.y / n
    out.z = q.z / n
    return out


def planar_yaw_from_quat(q: Quaternion) -> float:
    qn = quat_normalize(q)
    return 2.0 * math.atan2(qn.z, qn.w)


def yaw_to_quat(ros_yaw: float) -> Quaternion:
    half = ros_yaw * 0.5
    out = Quaternion()
    out.x = 0.0
    out.y = 0.0
    out.z = math.sin(half)
    out.w = math.cos(half)
    return out


def angle_wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def ned_to_cmd_pose(x_n: float, y_e: float, z_d: float, yaw_ned: float) -> PoseStamped:
    """Match offboard_controller._set_current_pose_from_ned."""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(y_e)
    p.pose.position.y = float(x_n)
    p.pose.position.z = float(-z_d)
    ros_yaw = -float(yaw_ned)
    p.pose.orientation = yaw_to_quat(ros_yaw)
    return p


def cmd_pose_planar_yaw(p: PoseStamped) -> float:
    return planar_yaw_from_quat(p.pose.orientation)


def px4_attitude_yaw_ned(msg: VehicleAttitude) -> float:
    q = msg.q
    return float(
        math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]),
        )
    )


def body_err_to_ned_horizontal(fwd: float, left: float, yaw_ned: float) -> Tuple[float, float]:
    c = math.cos(yaw_ned)
    s = math.sin(yaw_ned)
    d_north = fwd * c - left * s
    d_east = fwd * s + left * c
    return d_north, d_east


class ArucoLandingPlanner(Node):
    def __init__(self):
        super().__init__('aruco_landing_planner')

        # Defaults: very slow approach (tune per vehicle)
        self.declare_parameter('horizontal_speed_mps', 0.025)
        self.declare_parameter('vertical_speed_mps', 0.015)
        self.declare_parameter('yaw_rate_rad_s', 0.04)
        self.declare_parameter('control_period_s', 0.05)
        # No separate "freshness" window: last relative pose is trusted until this age without a new message.
        self.declare_parameter('marker_timeout_s', 5.0)

        self._h_speed = float(self.get_parameter('horizontal_speed_mps').value)
        self._v_speed = float(self.get_parameter('vertical_speed_mps').value)
        self._y_rate = float(self.get_parameter('yaw_rate_rad_s').value)
        self._period = float(self.get_parameter('control_period_s').value)
        self._marker_timeout_s = float(self.get_parameter('marker_timeout_s').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._x_n = 0.0
        self._y_e = 0.0
        self._z_d = 0.0
        self._yaw_ned = 0.0
        self._xy_valid = False
        self._z_valid = False

        self._rel: Optional[PoseStamped] = None
        self._rel_rx: Optional[rclpy.time.Time] = None

        self._smoothed: Optional[PoseStamped] = None
        self._frozen_target: Optional[PoseStamped] = None

        self.create_subscription(
            VehicleLocalPosition,
            '/drone/fmu/out/vehicle_local_position',
            self._cb_lp,
            qos,
        )
        self.create_subscription(
            VehicleAttitude,
            '/drone/fmu/out/vehicle_attitude',
            self._cb_att,
            qos,
        )
        self.create_subscription(PoseStamped, '/drone/aruco/relative_pose', self._cb_rel, 10)

        self._cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd_pose', 10)
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            'aruco_landing_planner: slow ramp X/Y/Z/yaw together -> /drone/cmd_pose'
        )

    def _cb_lp(self, msg: VehicleLocalPosition):
        self._xy_valid = bool(msg.xy_valid)
        self._z_valid = bool(msg.z_valid)
        if self._xy_valid:
            self._x_n = float(msg.x)
            self._y_e = float(msg.y)
        if self._z_valid:
            self._z_d = float(msg.z)

    def _cb_att(self, msg: VehicleAttitude):
        self._yaw_ned = px4_attitude_yaw_ned(msg)

    def _cb_rel(self, msg: PoseStamped):
        self._rel = msg
        self._rel_rx = self.get_clock().now()

    def _fmu_ok(self) -> bool:
        return self._xy_valid and self._z_valid

    def _fmu_cmd(self) -> PoseStamped:
        return ned_to_cmd_pose(self._x_n, self._y_e, self._z_d, self._yaw_ned)

    def _seconds_since_rel(self, now: rclpy.time.Time) -> float:
        if self._rel_rx is None:
            return float('inf')
        return (now - self._rel_rx).nanoseconds * 1e-9

    def _rel_trust(self, now: rclpy.time.Time) -> bool:
        """True if we still trust the last relative pose (same window as marker lost)."""
        if self._rel is None or self._rel_rx is None:
            return False
        return self._seconds_since_rel(now) <= self._marker_timeout_s

    def _marker_lost(self, now: rclpy.time.Time) -> bool:
        if self._rel_rx is None:
            return False
        return self._seconds_since_rel(now) > self._marker_timeout_s

    def _build_goal(self) -> Optional[PoseStamped]:
        if self._rel is None or not self._fmu_ok():
            return None
        base = self._fmu_cmd()
        fwd = float(self._rel.pose.position.x)
        left = float(self._rel.pose.position.y)
        dist = float(self._rel.pose.position.z)
        yaw_rel = planar_yaw_from_quat(self._rel.pose.orientation)

        dn, de = body_err_to_ned_horizontal(fwd, left, self._yaw_ned)
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x = base.pose.position.x + de
        g.pose.position.y = base.pose.position.y + dn
        # Detector z = range along optical axis (m); command toward that depth directly (no hover offset).
        g.pose.position.z = base.pose.position.z - dist

        ros_yaw_base = cmd_pose_planar_yaw(base)
        g.pose.orientation = yaw_to_quat(angle_wrap_pi(ros_yaw_base + yaw_rel))
        return g

    def _step_toward(self, cur: PoseStamped, goal: PoseStamped, dt: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()

        gx, gy, gz = goal.pose.position.x, goal.pose.position.y, goal.pose.position.z
        cx, cy, cz = cur.pose.position.x, cur.pose.position.y, cur.pose.position.z

        dx = gx - cx
        sx = math.copysign(min(abs(dx), self._h_speed * dt), dx)
        out.pose.position.x = cx + sx
        dy = gy - cy
        sy = math.copysign(min(abs(dy), self._h_speed * dt), dy)
        out.pose.position.y = cy + sy

        zdiff = gz - cz
        zstep = math.copysign(min(abs(zdiff), self._v_speed * dt), zdiff)
        out.pose.position.z = cz + zstep

        y_cur = cmd_pose_planar_yaw(cur)
        y_goal = cmd_pose_planar_yaw(goal)
        ydiff = angle_wrap_pi(y_goal - y_cur)
        ystep = math.copysign(min(abs(ydiff), self._y_rate * dt), ydiff)
        out.pose.orientation = yaw_to_quat(y_cur + ystep)
        return out

    def _tick(self):
        now = self.get_clock().now()
        dt = self._period

        if not self._fmu_ok():
            return

        if self._smoothed is None:
            self._smoothed = self._fmu_cmd()

        if self._marker_lost(now) and self._frozen_target is not None:
            self.get_logger().warn(
                'Marker timeout; clearing goal.',
                throttle_duration_sec=5.0,
            )
            self._frozen_target = None

        if self._rel_trust(now) and self._fmu_ok():
            g = self._build_goal()
            if g is not None:
                self._frozen_target = g

        goal = self._frozen_target
        if goal is None:
            self._smoothed = self._fmu_cmd()
            self._publish(self._smoothed)
            return

        self._smoothed = self._step_toward(self._smoothed, goal, dt)
        self._publish(self._smoothed)

    def _publish(self, p: PoseStamped):
        p.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
