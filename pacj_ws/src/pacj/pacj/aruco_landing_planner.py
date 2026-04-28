#!/usr/bin/env python3
"""
ArUco landing planner: publishes /cmd_pose from FMU state + ArUco relative PoseStamped.

Mirrors NED → /cmd_pose conventions in offboard_controller._set_current_pose_from_ned.
Run manually when ready, with offboard + tracking armed:

  ros2 run pacj aruco_landing_planner
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Phase(Enum):
    WAIT_FOR_MARKER = auto()
    ALIGN_XY_YAW = auto()
    DESCEND_TO_HOVER_OFFSET = auto()
    HOVER_TUNING = auto()
    BLIND_DESCEND_FMU_Z = auto()
    RETURN_TO_HOVER_AFTER_BLIND_FAIL = auto()
    RECOVER_TO_START = auto()


def quat_normalize(q: Quaternion) -> Quaternion:
    n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) or 1.0
    out = Quaternion()
    out.w = q.w / n
    out.x = q.x / n
    out.y = q.y / n
    out.z = q.z / n
    return out


def planar_yaw_from_quat(q: Quaternion) -> float:
    """Yaw (rad) for x=y=0 planar quaternion (same convention as offboard cmd pose)."""
    qn = quat_normalize(q)
    return 2.0 * math.atan2(qn.z, qn.w)


def yaw_to_quat(ros_yaw: float) -> Quaternion:
    half = ros_yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def clamp_scalar(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


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
    """Extract planar yaw (rad) from /cmd_pose quaternion."""
    return planar_yaw_from_quat(p.pose.orientation)


def px4_attitude_yaw_ned(msg: VehicleAttitude) -> float:
    """Same yaw extraction as offboard_controller.vehicle_attitude_cb."""
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


@dataclass
class FmuState:
    x_n: float
    y_e: float
    z_d: float
    yaw_ned: float
    xy_valid: bool
    z_valid: bool


class ArucoLandingPlanner(Node):
    def __init__(self):
        super().__init__('aruco_landing_planner')

        self.declare_parameter('hover_offset_m', 0.30)
        self.declare_parameter('align_xy_tolerance_m', 0.08)
        self.declare_parameter('align_yaw_tolerance_rad', 0.15)
        self.declare_parameter('align_stable_ticks', 8)
        self.declare_parameter('max_step_m', 0.35)
        self.declare_parameter('max_yaw_step_rad', 0.30)
        self.declare_parameter('descent_rate_mps', 0.10)
        self.declare_parameter('marker_timeout_s', 2.0)
        self.declare_parameter('marker_freshness_s', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('hover_tune_timeout_s', 12.0)
        self.declare_parameter('blind_landing_timeout_s', 25.0)
        self.declare_parameter('deck_delta_z_ned_m', 0.35)
        self.declare_parameter('blind_z_tol_m', 0.06)
        self.declare_parameter('hover_enter_hysteresis_m', 0.04)
        self.declare_parameter('hover_dwell_ticks', 10)
        self.declare_parameter('tune_stable_ticks', 10)
        self.declare_parameter('reacquire_stable_s', 0.35)
        self.declare_parameter('rel_z_gain', 1.0)

        self.hover_offset_m = float(self.get_parameter('hover_offset_m').value)
        self.align_xy_tol = float(self.get_parameter('align_xy_tolerance_m').value)
        self.align_yaw_tol = float(self.get_parameter('align_yaw_tolerance_rad').value)
        self.align_stable_ticks = int(self.get_parameter('align_stable_ticks').value)
        self.max_step_m = float(self.get_parameter('max_step_m').value)
        self.max_yaw_step_rad = float(self.get_parameter('max_yaw_step_rad').value)
        self.descent_rate_mps = float(self.get_parameter('descent_rate_mps').value)
        self.marker_timeout_s = float(self.get_parameter('marker_timeout_s').value)
        self.marker_freshness_s = float(self.get_parameter('marker_freshness_s').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.hover_tune_timeout_s = float(self.get_parameter('hover_tune_timeout_s').value)
        self.blind_landing_timeout_s = float(self.get_parameter('blind_landing_timeout_s').value)
        self.deck_delta_z_ned_m = float(self.get_parameter('deck_delta_z_ned_m').value)
        self.blind_z_tol_m = float(self.get_parameter('blind_z_tol_m').value)
        self.hover_enter_hysteresis_m = float(self.get_parameter('hover_enter_hysteresis_m').value)
        self.hover_dwell_ticks = int(self.get_parameter('hover_dwell_ticks').value)
        self.tune_stable_ticks = int(self.get_parameter('tune_stable_ticks').value)
        self.reacquire_stable_s = float(self.get_parameter('reacquire_stable_s').value)
        self.rel_z_gain = float(self.get_parameter('rel_z_gain').value)

        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._fmu = FmuState(0.0, 0.0, 0.0, 0.0, False, False)
        self._rel: Optional[PoseStamped] = None
        self._rel_rx_time: Optional[rclpy.time.Time] = None

        self._phase = Phase.WAIT_FOR_MARKER
        self._cmd = PoseStamped()
        self._cmd.header.frame_id = 'map'

        self._start_pose_cmd = PoseStamped()
        self._hover_pose_cmd = PoseStamped()
        self._start_pose_latched = False

        self._align_ok_ticks = 0
        self._hover_dwell_ticks = 0
        self._tune_ok_ticks = 0
        self._hover_tune_deadline: Optional[rclpy.time.Time] = None
        self._blind_deadline: Optional[rclpy.time.Time] = None
        self._z_land_ned = 0.0
        self._reacquire_since: Optional[rclpy.time.Time] = None

        self.create_subscription(
            VehicleLocalPosition,
            '/drone/fmu/out/vehicle_local_position',
            self._cb_local,
            qos_best,
        )
        self.create_subscription(
            VehicleAttitude,
            '/drone/fmu/out/vehicle_attitude',
            self._cb_attitude,
            qos_best,
        )
        self.create_subscription(PoseStamped, '/drone/aruco/relative_pose', self._cb_rel, 10)

        self._cmd_pub = self.create_publisher(PoseStamped, '/cmd_pose', 10)

        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            'aruco_landing_planner: stream /cmd_pose (OFFBOARD + tracking armed in offboard_controller).'
        )

    def _cb_local(self, msg: VehicleLocalPosition):
        self._fmu.xy_valid = bool(msg.xy_valid)
        self._fmu.z_valid = bool(msg.z_valid)
        if self._fmu.xy_valid:
            self._fmu.x_n = float(msg.x)
            self._fmu.y_e = float(msg.y)
        if self._fmu.z_valid:
            self._fmu.z_d = float(msg.z)

    def _cb_attitude(self, msg: VehicleAttitude):
        self._fmu.yaw_ned = px4_attitude_yaw_ned(msg)

    def _cb_rel(self, msg: PoseStamped):
        self._rel = msg
        self._rel_rx_time = self.get_clock().now()

    def _fmu_ok(self) -> bool:
        return self._fmu.xy_valid and self._fmu.z_valid

    def _fmu_cmd_pose(self) -> PoseStamped:
        return ned_to_cmd_pose(self._fmu.x_n, self._fmu.y_e, self._fmu.z_d, self._fmu.yaw_ned)

    def _seconds_since_rel(self, now: rclpy.time.Time) -> float:
        if self._rel_rx_time is None:
            return 1e9
        return (now - self._rel_rx_time).nanoseconds * 1e-9

    def _rel_fresh(self, now: rclpy.time.Time) -> bool:
        if self._rel is None:
            return False
        return self._seconds_since_rel(now) <= self.marker_freshness_s

    def _marker_lost(self, now: rclpy.time.Time) -> bool:
        return self._seconds_since_rel(now) > self.marker_timeout_s

    def _rel_xy_yaw_err(self) -> Tuple[float, float]:
        """Horizontal magnitude (m) and |yaw error| (rad) from detector message."""
        if self._rel is None:
            return 1e9, math.pi
        fwd = float(self._rel.pose.position.x)
        left = float(self._rel.pose.position.y)
        xy = math.hypot(fwd, left)
        yaw_e = abs(planar_yaw_from_quat(self._rel.pose.orientation))
        return xy, yaw_e

    def _hover_range_err_m(self) -> float:
        if self._rel is None:
            return 1e9
        return float(self._rel.pose.position.z) - self.hover_offset_m

    def _apply_step_toward(self, cur: PoseStamped, target: PoseStamped) -> PoseStamped:
        out = PoseStamped()
        out.header = cur.header
        out.header.stamp = self.get_clock().now().to_msg()
        dx = target.pose.position.x - cur.pose.position.x
        dy = target.pose.position.y - cur.pose.position.y
        dz = target.pose.position.z - cur.pose.position.z
        horiz = math.hypot(dx, dy)
        if horiz > self.max_step_m and horiz > 1e-9:
            s = self.max_step_m / horiz
            dx *= s
            dy *= s
        dz = clamp_scalar(dz, -self.max_step_m, self.max_step_m)
        out.pose.position.x = cur.pose.position.x + dx
        out.pose.position.y = cur.pose.position.y + dy
        out.pose.position.z = cur.pose.position.z + dz

        yaw_cur = cmd_pose_planar_yaw(cur)
        yaw_tgt = cmd_pose_planar_yaw(target)
        dyaw = angle_wrap_pi(yaw_tgt - yaw_cur)
        dyaw = clamp_scalar(dyaw, -self.max_yaw_step_rad, self.max_yaw_step_rad)
        out.pose.orientation = yaw_to_quat(yaw_cur + dyaw)
        return out

    def _vision_target_pose(self) -> PoseStamped:
        """FMU cmd pose + body-relative horizontal + yaw + Z from range error."""
        base = self._fmu_cmd_pose()
        if self._rel is None:
            return base

        fwd = float(self._rel.pose.position.x)
        left = float(self._rel.pose.position.y)
        dist = float(self._rel.pose.position.z)
        yaw_rel = planar_yaw_from_quat(self._rel.pose.orientation)

        dn, de = body_err_to_ned_horizontal(fwd, left, self._fmu.yaw_ned)
        tgt = PoseStamped()
        tgt.header = base.header
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.pose.position.x = base.pose.position.x + de
        tgt.pose.position.y = base.pose.position.y + dn
        rel_z_err = (dist - self.hover_offset_m) * self.rel_z_gain
        tgt.pose.position.z = base.pose.position.z - rel_z_err

        ros_yaw_base = cmd_pose_planar_yaw(base)
        tgt.pose.orientation = yaw_to_quat(angle_wrap_pi(ros_yaw_base + yaw_rel))
        return tgt

    def _align_only_target(self, hold_z: float) -> PoseStamped:
        base = self._fmu_cmd_pose()
        if self._rel is None:
            out = base
            out.pose.position.z = hold_z
            return out
        fwd = float(self._rel.pose.position.x)
        left = float(self._rel.pose.position.y)
        yaw_rel = planar_yaw_from_quat(self._rel.pose.orientation)
        dn, de = body_err_to_ned_horizontal(fwd, left, self._fmu.yaw_ned)
        tgt = PoseStamped()
        tgt.header = base.header
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.pose.position.x = base.pose.position.x + de
        tgt.pose.position.y = base.pose.position.y + dn
        tgt.pose.position.z = hold_z
        ros_yaw_base = cmd_pose_planar_yaw(base)
        tgt.pose.orientation = yaw_to_quat(angle_wrap_pi(ros_yaw_base + yaw_rel))
        return tgt

    def _hover_hold_target(self) -> PoseStamped:
        v = self._vision_target_pose()
        out = PoseStamped()
        out.header = v.header
        out.pose.position.x = v.pose.position.x
        out.pose.position.y = v.pose.position.y
        out.pose.position.z = self._hover_pose_cmd.pose.position.z
        out.pose.orientation = v.pose.orientation
        return out

    def _set_phase(self, phase: Phase, now: rclpy.time.Time):
        self.get_logger().info(f'phase -> {phase.name}')
        self._phase = phase
        if phase == Phase.HOVER_TUNING:
            self._hover_tune_deadline = now + rclpy.duration.Duration(seconds=self.hover_tune_timeout_s)
            self._tune_ok_ticks = 0
        if phase == Phase.BLIND_DESCEND_FMU_Z:
            self._blind_deadline = now + rclpy.duration.Duration(seconds=self.blind_landing_timeout_s)
            z_start = self._fmu.z_d
            self._z_land_ned = z_start + self.deck_delta_z_ned_m

    def _pose_xy_close(self, a: PoseStamped, b: PoseStamped, tol: float) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy) < tol

    def _pose_close(self, a: PoseStamped, b: PoseStamped, pos_tol: float, yaw_tol: float) -> bool:
        if not self._pose_xy_close(a, b, pos_tol):
            return False
        if abs(a.pose.position.z - b.pose.position.z) > pos_tol:
            return False
        dyaw = abs(angle_wrap_pi(cmd_pose_planar_yaw(a) - cmd_pose_planar_yaw(b)))
        return dyaw < yaw_tol

    def _tick(self):
        now = self.get_clock().now()
        if not self._fmu_ok():
            return

        if self._cmd.header.stamp.sec == 0 and self._cmd.header.stamp.nanosec == 0:
            self._cmd = self._fmu_cmd_pose()

        # --- WAIT_FOR_MARKER ---
        if self._phase == Phase.WAIT_FOR_MARKER:
            self._cmd = self._fmu_cmd_pose()
            if self._rel_fresh(now):
                if not self._start_pose_latched:
                    self._start_pose_cmd = self._fmu_cmd_pose()
                    self._start_pose_latched = True
                self._set_phase(Phase.ALIGN_XY_YAW, now)
                self._align_ok_ticks = 0
            self._publish_cmd()
            return

        # --- ALIGN_XY_YAW ---
        if self._phase == Phase.ALIGN_XY_YAW:
            hold_z = self._cmd.pose.position.z
            tgt = self._align_only_target(hold_z)
            self._cmd = self._apply_step_toward(self._cmd, tgt)
            xy, yaw_abs = self._rel_xy_yaw_err() if self._rel_fresh(now) else (1e9, math.pi)
            if self._rel_fresh(now) and xy < self.align_xy_tol and yaw_abs < self.align_yaw_tol:
                self._align_ok_ticks += 1
            else:
                self._align_ok_ticks = 0
            if self._align_ok_ticks >= self.align_stable_ticks:
                self._set_phase(Phase.DESCEND_TO_HOVER_OFFSET, now)
                self._hover_dwell_ticks = 0
            self._publish_cmd()
            return

        # --- DESCEND_TO_HOVER_OFFSET ---
        if self._phase == Phase.DESCEND_TO_HOVER_OFFSET:
            if self._marker_lost(now):
                self._set_phase(Phase.RECOVER_TO_START, now)
                self._reacquire_since = None
                self._publish_cmd()
                return
            tgt = self._vision_target_pose()
            self._cmd = self._apply_step_toward(self._cmd, tgt)
            he = self._hover_range_err_m()
            if abs(he) < self.hover_enter_hysteresis_m:
                self._hover_dwell_ticks += 1
            else:
                self._hover_dwell_ticks = 0
            if self._hover_dwell_ticks >= self.hover_dwell_ticks:
                self._hover_pose_cmd = PoseStamped()
                self._hover_pose_cmd.header = self._cmd.header
                self._hover_pose_cmd.pose = self._cmd.pose
                self._set_phase(Phase.HOVER_TUNING, now)
            self._publish_cmd()
            return

        # --- HOVER_TUNING ---
        if self._phase == Phase.HOVER_TUNING:
            if self._marker_lost(now):
                self._set_phase(Phase.RECOVER_TO_START, now)
                self._reacquire_since = None
                self._publish_cmd()
                return
            if self._hover_tune_deadline is not None and now > self._hover_tune_deadline:
                self._cmd = self._apply_step_toward(self._cmd, self._hover_pose_cmd)
                self._set_phase(Phase.HOVER_TUNING, now)
                self._publish_cmd()
                return
            tgt = self._hover_hold_target()
            self._cmd = self._apply_step_toward(self._cmd, tgt)
            xy, yaw_abs = self._rel_xy_yaw_err() if self._rel_fresh(now) else (1e9, math.pi)
            if self._rel_fresh(now) and xy < self.align_xy_tol and yaw_abs < self.align_yaw_tol:
                self._tune_ok_ticks += 1
            else:
                self._tune_ok_ticks = 0
            if self._tune_ok_ticks >= self.tune_stable_ticks:
                self._set_phase(Phase.BLIND_DESCEND_FMU_Z, now)
            self._publish_cmd()
            return

        # --- BLIND_DESCEND_FMU_Z ---
        if self._phase == Phase.BLIND_DESCEND_FMU_Z:
            if self._blind_deadline is not None and now > self._blind_deadline:
                self._set_phase(Phase.RETURN_TO_HOVER_AFTER_BLIND_FAIL, now)
                self._publish_cmd()
                return

            step_z = self.descent_rate_mps / max(self.control_rate_hz, 1e-3)
            tgt = PoseStamped()
            tgt.header = self._cmd.header
            tgt.header.stamp = self.get_clock().now().to_msg()
            tgt.pose.position.x = self._hover_pose_cmd.pose.position.x
            tgt.pose.position.y = self._hover_pose_cmd.pose.position.y
            tgt.pose.position.z = self._cmd.pose.position.z - step_z
            tgt.pose.orientation = self._hover_pose_cmd.pose.orientation
            self._cmd = self._apply_step_toward(self._cmd, tgt)

            if self._fmu.z_d >= self._z_land_ned - self.blind_z_tol_m:
                self.get_logger().info('blind landing: FMU z within tolerance of deck depth')
                self._set_phase(Phase.WAIT_FOR_MARKER, now)
            self._publish_cmd()
            return

        # --- RETURN_TO_HOVER_AFTER_BLIND_FAIL ---
        if self._phase == Phase.RETURN_TO_HOVER_AFTER_BLIND_FAIL:
            self._cmd = self._apply_step_toward(self._cmd, self._hover_pose_cmd)
            if self._pose_close(self._cmd, self._hover_pose_cmd, self.align_xy_tol, self.align_yaw_tol):
                self._set_phase(Phase.HOVER_TUNING, now)
            self._publish_cmd()
            return

        # --- RECOVER_TO_START ---
        if self._phase == Phase.RECOVER_TO_START:
            if self._rel_fresh(now):
                if self._reacquire_since is None:
                    self._reacquire_since = now
                elif (now - self._reacquire_since).nanoseconds * 1e-9 >= self.reacquire_stable_s:
                    self._set_phase(Phase.ALIGN_XY_YAW, now)
                    self._align_ok_ticks = 0
                    self._reacquire_since = None
                    self._publish_cmd()
                    return
            else:
                self._reacquire_since = None

            self._cmd = self._apply_step_toward(self._cmd, self._start_pose_cmd)
            if self._pose_close(self._cmd, self._start_pose_cmd, self.align_xy_tol, self.align_yaw_tol):
                self._set_phase(Phase.WAIT_FOR_MARKER, now)
                self._start_pose_latched = False
            self._publish_cmd()
            return

    def _publish_cmd(self):
        self._cmd.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(self._cmd)


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
