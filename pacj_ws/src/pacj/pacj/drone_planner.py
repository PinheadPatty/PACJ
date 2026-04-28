"""RTAB-Map aligned planner: 2D A* on occupancy grid when available, else straight line."""

import heapq
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import Buffer, TransformException, TransformListener

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _shortest_yaw_delta(a: float, b: float) -> float:
    d = b - a
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def _dist3(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    dz = a.pose.position.z - b.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class DronePlanner(Node):
    def __init__(self):
        super().__init__('drone_planner')

        self.map_frame = self.declare_parameter('map_frame', 'drone/map').value
        self.base_frame = self.declare_parameter('base_frame', 'drone/drone_link').value
        occupancy_topic = self.declare_parameter('occupancy_topic', '/drone/grid_map').value
        self.waypoint_spacing_m = float(self.declare_parameter('waypoint_spacing_m', 0.35).value)
        self.waypoint_accept_radius_m = float(
            self.declare_parameter('waypoint_accept_radius_m', 0.22).value
        )
        self.execution_period_s = float(self.declare_parameter('execution_period_s', 0.05).value)
        self.occupancy_threshold = int(self.declare_parameter('occupancy_threshold', 50).value)
        self.reject_unknown_cells = bool(self.declare_parameter('reject_unknown_cells', False).value)
        self.astar_max_expansions = int(self.declare_parameter('astar_max_expansions', 500_000).value)
        self.snap_to_free_radius_cells = int(self.declare_parameter('snap_to_free_radius_cells', 40).value)

        # Maximum distance ahead of the drone's current position that a /cmd_pose
        # target may be placed. This is the only velocity-limiting mechanism; the
        # offboard controller applies no clamping of its own.
        self.lookahead_distance_m = float(self.declare_parameter('lookahead_distance_m', 0.5).value)

        self.cmd_pub = self.create_publisher(PoseStamped, '/cmd_pose', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose_3d', self.goal_cb, 10
        )

        self._grid = None
        if occupancy_topic:
            self.create_subscription(OccupancyGrid, occupancy_topic, self._grid_cb, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._waypoints = []
        self._wp_index = 0

        self.exec_timer = self.create_timer(self.execution_period_s, self._execution_timer_cb)

        self.get_logger().info(
            f"Drone planner: map={self.map_frame} base={self.base_frame} "
            f"spacing={self.waypoint_spacing_m}m accept={self.waypoint_accept_radius_m}m "
            f"lookahead={self.lookahead_distance_m}m "
            f"(2D A* when occupancy grid is present)"
        )

    def _clear_plan(self):
        self._waypoints = []
        self._wp_index = 0

    def _grid_cb(self, msg: OccupancyGrid):
        self._grid = msg

    def _get_pose_map_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF {self.map_frame} <- {self.base_frame}: {ex}")
            return None

        ps = PoseStamped()
        ps.header.stamp = t.header.stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        return ps

    def _transform_goal_to_map(self, msg: PoseStamped) -> PoseStamped:
        frame = msg.header.frame_id
        if not frame or frame == self.map_frame:
            return msg
        try:
            tf_map = self.tf_buffer.lookup_transform(
                self.map_frame, frame, Time(), timeout=Duration(seconds=1.0)
            )
            return do_transform_pose(msg, tf_map)
        except TransformException as ex:
            self.get_logger().error(f"Goal transform {frame} -> {self.map_frame}: {ex}")
            raise

    def _world_xy_to_grid_idx(self, wx: float, wy: float):
        if self._grid is None:
            return None, None
        g = self._grid
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        res = g.info.resolution
        if res < 1e-9:
            return None, None
        mx = (wx - ox) / res
        my = (wy - oy) / res
        ix = int(mx)
        iy = int(my)
        if ix < 0 or iy < 0 or ix >= g.info.width or iy >= g.info.height:
            return None, None
        return ix, iy

    def _grid_value(self, ix: int, iy: int):
        g = self._grid
        idx = iy * g.info.width + ix
        if idx < 0 or idx >= len(g.data):
            return None
        return g.data[idx]

    def _cell_traversable(self, ix: int, iy: int) -> bool:
        g = self._grid
        if g is None:
            return False
        if ix < 0 or iy < 0 or ix >= g.info.width or iy >= g.info.height:
            return False
        v = self._grid_value(ix, iy)
        if v is None:
            return not self.reject_unknown_cells
        if v < 0:
            return not self.reject_unknown_cells
        return v < self.occupancy_threshold

    def _grid_center_world(self, ix: int, iy: int):
        g = self._grid
        res = g.info.resolution
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        wx = ox + (ix + 0.5) * res
        wy = oy + (iy + 0.5) * res
        return wx, wy

    def _snap_to_free_cell(self, ix: int, iy: int):
        """Nearest traversable cell within Chebyshev radius."""
        if self._cell_traversable(ix, iy):
            return ix, iy
        g = self._grid
        rmax = self.snap_to_free_radius_cells
        for r in range(1, rmax + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    tx, ty = ix + dx, iy + dy
                    if self._cell_traversable(tx, ty):
                        return tx, ty
        return None, None

    def _astar_2d(self, s_ix: int, s_iy: int, g_ix: int, g_iy: int):
        """8-connected grid A*. Returns list of (ix, iy) from start to goal inclusive."""
        if not self._cell_traversable(s_ix, s_iy) or not self._cell_traversable(g_ix, g_iy):
            return None

        g = self._grid
        w, h = g.info.width, g.info.height
        res = max(g.info.resolution, 1e-6)
        sqrt2 = 1.4142135623730951

        def h_fun(ix, iy):
            dx = ix - g_ix
            dy = iy - g_iy
            return math.hypot(dx, dy) * res

        open_heap = []
        heapq.heappush(open_heap, (h_fun(s_ix, s_iy), 0.0, s_ix, s_iy))
        came = {}
        gscore = {(s_ix, s_iy): 0.0}
        expansions = 0

        while open_heap:
            _, gc, ix, iy = heapq.heappop(open_heap)
            expansions += 1
            if expansions > self.astar_max_expansions:
                return None
            if gc > gscore.get((ix, iy), float('inf')) + 1e-9:
                continue
            if ix == g_ix and iy == g_iy:
                rev = []
                cur = (ix, iy)
                while cur != (s_ix, s_iy):
                    rev.append(cur)
                    cur = came[cur]
                rev.append((s_ix, s_iy))
                rev.reverse()
                return rev

            for dx, dy in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                nx, ny = ix + dx, iy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if not self._cell_traversable(nx, ny):
                    continue
                step = res * (sqrt2 if dx != 0 and dy != 0 else 1.0)
                ng = gc + step
                if ng < gscore.get((nx, ny), float('inf')):
                    came[(nx, ny)] = (ix, iy)
                    gscore[(nx, ny)] = ng
                    f = ng + h_fun(nx, ny)
                    heapq.heappush(open_heap, (f, ng, nx, ny))
        return None

    def _densify_xy_polyline(self, xy_points: list):
        """Resample polyline in XY so consecutive points are ~waypoint_spacing_m apart."""
        if len(xy_points) < 2:
            return list(xy_points)
        spacing = max(self.waypoint_spacing_m, 0.05)
        chain = [xy_points[0]]
        for i in range(len(xy_points) - 1):
            ax, ay = xy_points[i]
            bx, by = xy_points[i + 1]
            dx = bx - ax
            dy = by - ay
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-9:
                continue
            n = max(1, int(math.ceil(seg_len / spacing)))
            for j in range(1, n + 1):
                t = j / float(n)
                chain.append((ax + t * dx, ay + t * dy))
        dedup = [chain[0]]
        for p in chain[1:]:
            if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 0.02:
                dedup.append(p)
        return dedup

    def _build_poses_from_xy_path(
        self, xy_path: list, start: PoseStamped, end: PoseStamped
    ) -> list:
        """Z varies linearly with distance along the XY polyline; yaw aligns with motion, goal sets final orientation."""
        sz = start.pose.position.z
        ez = end.pose.position.z
        if len(xy_path) < 1:
            return []

        if len(xy_path) == 1:
            p = PoseStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = self.map_frame
            p.pose.position.x = xy_path[0][0]
            p.pose.position.y = xy_path[0][1]
            p.pose.position.z = ez
            p.pose.orientation = end.pose.orientation
            return [p]

        total_xy = 0.0
        for i in range(len(xy_path) - 1):
            ax, ay = xy_path[i]
            bx, by = xy_path[i + 1]
            total_xy += math.hypot(bx - ax, by - ay)
        if total_xy < 1e-6:
            total_xy = 1e-6

        stamp = self.get_clock().now().to_msg()
        out = []
        cum = 0.0
        for i, (px, py) in enumerate(xy_path):
            if i > 0:
                px0, py0 = xy_path[i - 1]
                cum += math.hypot(px - px0, py - py0)
            pz = sz + (cum / total_xy) * (ez - sz)
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = self.map_frame
            p.pose.position.x = px
            p.pose.position.y = py
            p.pose.position.z = pz
            if i < len(xy_path) - 1:
                nx, ny = xy_path[i + 1]
                yaw = math.atan2(ny - py, nx - px)
                p.pose.orientation = _quaternion_from_yaw(yaw)
            else:
                p.pose.orientation = end.pose.orientation
            out.append(p)
        return out

    def _plan_with_astar(self, start: PoseStamped, end: PoseStamped):
        """Return list of PoseStamped or None if planning failed / no grid."""
        if self._grid is None:
            return None

        s_ix, s_iy = self._world_xy_to_grid_idx(start.pose.position.x, start.pose.position.y)
        g_ix, g_iy = self._world_xy_to_grid_idx(end.pose.position.x, end.pose.position.y)
        if s_ix is None or g_ix is None:
            return None

        s_ix, s_iy = self._snap_to_free_cell(s_ix, s_iy)
        g_ix, g_iy = self._snap_to_free_cell(g_ix, g_iy)
        if s_ix is None or g_ix is None:
            self.get_logger().warn("A*: could not snap start/goal to a free grid cell.")
            return None

        cell_path = self._astar_2d(s_ix, s_iy, g_ix, g_iy)
        if cell_path is None:
            self.get_logger().warn("A*: no path found.")
            return None

        xy_rough = [self._grid_center_world(ix, iy) for ix, iy in cell_path]
        xy_dense = self._densify_xy_polyline(xy_rough)
        self.get_logger().info(
            f"A*: grid cells={len(cell_path)} densified XY points={len(xy_dense)}"
        )
        return self._build_poses_from_xy_path(xy_dense, start, end)

    def _segment_blocked(self, ax: float, ay: float, bx: float, by: float) -> bool:
        if self._grid is None:
            return False
        dx = bx - ax
        dy = by - ay
        steps = max(2, int(math.hypot(dx, dy) / max(self._grid.info.resolution * 0.5, 0.05)))
        for i in range(steps + 1):
            t = i / float(steps)
            px = ax + t * dx
            py = ay + t * dy
            ix, iy = self._world_xy_to_grid_idx(px, py)
            if ix is None:
                if self.reject_unknown_cells:
                    return True
                continue
            v = self._grid_value(ix, iy)
            if v is None:
                if self.reject_unknown_cells:
                    return True
                continue
            if v < 0:
                if self.reject_unknown_cells:
                    return True
                continue
            if v >= self.occupancy_threshold:
                return True
        return False

    def _plan_blocked(self, poses: list) -> bool:
        if len(poses) < 2:
            return False
        for i in range(len(poses) - 1):
            a = poses[i].pose.position
            b = poses[i + 1].pose.position
            if self._segment_blocked(a.x, a.y, b.x, b.y):
                return True
        return False

    def _build_waypoints(self, start: PoseStamped, end: PoseStamped) -> list:
        sx = start.pose.position.x
        sy = start.pose.position.y
        sz = start.pose.position.z
        ex = end.pose.position.x
        ey = end.pose.position.y
        ez = end.pose.position.z
        dx = ex - sx
        dy = ey - sy
        dz = ez - sz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 1e-6:
            return [end]

        n = max(1, int(math.ceil(dist / max(self.waypoint_spacing_m, 0.05))))
        ys = _yaw_from_quaternion(
            start.pose.orientation.x,
            start.pose.orientation.y,
            start.pose.orientation.z,
            start.pose.orientation.w,
        )
        ye = _yaw_from_quaternion(
            end.pose.orientation.x,
            end.pose.orientation.y,
            end.pose.orientation.z,
            end.pose.orientation.w,
        )
        d_yaw = _shortest_yaw_delta(ys, ye)

        out = []
        stamp = self.get_clock().now().to_msg()
        for i in range(n + 1):
            t = i / float(n)
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = self.map_frame
            p.pose.position.x = sx + t * dx
            p.pose.position.y = sy + t * dy
            p.pose.position.z = sz + t * dz
            yaw = ys + t * d_yaw
            p.pose.orientation = _quaternion_from_yaw(yaw)
            out.append(p)
        return out

    def goal_cb(self, msg: PoseStamped):
        start = self._get_pose_map_from_tf()
        if start is None:
            self.get_logger().error("No TF pose; is RTAB-Map running (map -> base)?")
            return

        try:
            goal_map = self._transform_goal_to_map(msg)
        except TransformException:
            return

        poses = self._plan_with_astar(start, goal_map)
        plan_mode = "2d_astar"
        if poses is None:
            plan_mode = "straight_line"
            poses = self._build_waypoints(start, goal_map)
            if self._plan_blocked(poses):
                self.get_logger().warn("Rejected plan: line crosses occupied / unknown cells.")
                return

        self._waypoints = poses
        self._wp_index = 0

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        path.poses = poses
        self.path_pub.publish(path)

        self.get_logger().info(
            f"Executing {len(poses)} waypoints ({plan_mode}), "
            f"track length {self._track_length(poses):.2f} m"
        )

    @staticmethod
    def _track_length(poses):
        if len(poses) < 2:
            return 0.0
        s = 0.0
        for i in range(len(poses) - 1):
            a = poses[i].pose.position
            b = poses[i + 1].pose.position
            s += math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
        return s

    def _advance_waypoint_index(self, cur: PoseStamped):
        """Pure-pursuit advancement: greedily skip waypoints the drone has already passed.

        A waypoint is considered passed when the *next* waypoint is strictly
        closer than the current one, meaning the drone is geometrically past the
        current target. This produces smooth continuous tracking without waiting
        at each point.
        """
        while self._wp_index < len(self._waypoints) - 1:
            cur_tgt = self._waypoints[self._wp_index]
            nxt_tgt = self._waypoints[self._wp_index + 1]
            dist_cur = _dist3(cur, cur_tgt)
            dist_nxt = _dist3(cur, nxt_tgt)
            if dist_nxt < dist_cur:
                self._wp_index += 1
            else:
                break

    def _clamp_lookahead(self, cur: PoseStamped, tgt: PoseStamped) -> PoseStamped:
        """If tgt is farther than lookahead_distance_m from cur, clamp it along the line."""
        cx = cur.pose.position.x
        cy = cur.pose.position.y
        cz = cur.pose.position.z
        tx = tgt.pose.position.x
        ty = tgt.pose.position.y
        tz = tgt.pose.position.z
        dx = tx - cx
        dy = ty - cy
        dz = tz - cz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= self.lookahead_distance_m or dist < 1e-6:
            return tgt

        scale = self.lookahead_distance_m / dist
        clamped = PoseStamped()
        clamped.header = tgt.header
        clamped.pose.position.x = cx + dx * scale
        clamped.pose.position.y = cy + dy * scale
        clamped.pose.position.z = cz + dz * scale
        clamped.pose.orientation = tgt.pose.orientation
        return clamped

    def _execution_timer_cb(self):
        """Stream /cmd_pose every tick while a goal is active.

        On transient TF outages this callback skips publishing for that tick.
        With persistent outages, offboard_controller's cmd_timeout_s watchdog
        should return control to HOLD_LOCAL.
        """
        if not self._waypoints:
            return

        cur = self._get_pose_map_from_tf()
        if cur is None:
            # TF unavailable; do not publish this tick (timeout will catch a
            # persistent failure, but transient TF gaps should not trigger it).
            return

        # Advance index using pure-pursuit logic.
        self._advance_waypoint_index(cur)

        # Check if we have reached the final waypoint.
        tgt = self._waypoints[self._wp_index]
        is_last = (self._wp_index == len(self._waypoints) - 1)
        if is_last and _dist3(cur, tgt) < self.waypoint_accept_radius_m:
            self.get_logger().info("Waypoint execution finished.")
            self._clear_plan()
            return

        # Clamp the lookahead distance and publish every tick.
        cmd = self._clamp_lookahead(cur, tgt)
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)


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