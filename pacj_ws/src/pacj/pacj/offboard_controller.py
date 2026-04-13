#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleAttitude, VehicleLocalPosition

import cv2
from cv_bridge import CvBridge
import numpy as np

class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers to PX4
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers from PX4
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_cb, qos_profile)
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_pos_cb, qos_profile)

        # Subscribers for Teleop and Landing
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.cmd_pose_sub = self.create_subscription(PoseStamped, '/cmd_pose', self.cmd_pose_cb, 10)
        self.start_landing_sub = self.create_subscription(Bool, '/start_landing', self.start_landing_cb, 10)
        self.rover_pose_sub = self.create_subscription(PoseStamped, '/rover_pose', self.rover_pose_cb, 10)
        self.landing_success_pub = self.create_publisher(Bool, '/landing_success', 10)

        # Subscribers for Camera
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = self.declare_parameter('marker_size', 0.046).value
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_yaw = 0.0
        self.drone_pos = [0.0, 0.0, 0.0]
        
        self.rover_pos_ned = [0.0, 0.0, 0.0]
        self.rover_yaw_ned = 0.0
        self.rover_pose_received = False

        self.aruco_rel_pos = [0.0, 0.0, 0.0]
        self.aruco_rel_yaw = 0.0
        self.aruco_last_seen = self.get_clock().now()
        self.aruco_visible = False

        # Modes: 'VELOCITY', 'POSITION', 'LANDING'
        self.control_mode = 'VELOCITY'
        self.current_twist = Twist()
        self.current_pose = PoseStamped()
        
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 2.0
        
        # Landing State Machine: SLAM_APPROACH, ARUCO_HOVER, ARUCO_DESCENT, READY_TO_COUPLE
        self.land_state = 'SLAM_APPROACH'

        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info("Offboard Controller Initialized.")

    def vehicle_attitude_cb(self, msg):
        q = msg.q
        self.current_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

    def vehicle_local_pos_cb(self, msg):
        self.drone_pos = [msg.x, msg.y, msg.z]

    def vehicle_status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmd_vel_cb(self, msg):
        if self.control_mode != 'LANDING':
            self.control_mode = 'VELOCITY'
            self.current_twist = msg

    def cmd_pose_cb(self, msg):
        if self.control_mode != 'LANDING':
            self.control_mode = 'POSITION'
            self.current_pose = msg

    def start_landing_cb(self, msg):
        if msg.data and self.control_mode != 'LANDING':
            self.control_mode = 'LANDING'
            self.land_state = 'SLAM_APPROACH'
            self.get_logger().info("LANDING MODE ACTIVATED: Phase 1 (SLAM Approach)")
        elif not msg.data and self.control_mode == 'LANDING':
            self.control_mode = 'VELOCITY'
            self.get_logger().info("LANDING MODE DEACTIVATED")

    def rover_pose_cb(self, msg):
        # Convert ENU to NED
        ros_x = float(msg.pose.position.x)
        ros_y = float(msg.pose.position.y)
        ros_z = float(msg.pose.position.z)
        
        self.rover_pos_ned[0] = ros_y
        self.rover_pos_ned[1] = ros_x
        self.rover_pos_ned[2] = -ros_z

        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w
        
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        target_yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        self.rover_yaw_ned = -target_yaw_enu
        self.rover_pose_received = True

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        if self.control_mode != 'LANDING' or self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            marker_points = np.array([
                [-self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            _, rvec, tvec = cv2.solvePnP(marker_points, corners[0][0], self.camera_matrix, self.dist_coeffs)
            tvec = tvec.flatten()
            
            # Camera -> NED body
            # Camera X = East, Y = -North, Z = Down
            self.aruco_rel_pos[0] = -tvec[1]
            self.aruco_rel_pos[1] = tvec[0]
            self.aruco_rel_pos[2] = tvec[2]

            # Extract Yaw from rvec
            rmat, _ = cv2.Rodrigues(rvec)
            # R_ned_c transforms Camera frame to NED frame
            R_ned_c = np.array([
                [0, -1,  0],
                [1,  0,  0],
                [0,  0,  1]
            ])
            R_ned_m = R_ned_c @ rmat
            # Yaw is atan2(R_yx, R_xx)
            self.aruco_rel_yaw = math.atan2(R_ned_m[1, 0], R_ned_m[0, 0])

            self.aruco_last_seen = self.get_clock().now()
            self.aruco_visible = True
        else:
            if (self.get_clock().now() - self.aruco_last_seen).nanoseconds > 1e9: # 1 second timeout
                self.aruco_visible = False

    def timer_cb(self):
        self.publish_offboard_control_mode()

        msg = TrajectorySetpoint()
        
        if self.control_mode == 'VELOCITY':
            # Velocity Control Mode
            xy_multiplier = 5.0
            z_multiplier = 1.0
            
            body_forward = float(self.current_twist.linear.x) * xy_multiplier
            body_right = -float(self.current_twist.linear.y) * xy_multiplier
            body_down = -float(self.current_twist.linear.z) * z_multiplier
            
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            
            msg.velocity[0] = body_forward * cos_yaw - body_right * sin_yaw
            msg.velocity[1] = body_forward * sin_yaw + body_right * cos_yaw
            msg.velocity[2] = body_down
            msg.yawspeed = -float(self.current_twist.angular.z)

            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.jerk = [float('nan'), float('nan'), float('nan')]
            msg.yaw = float('nan')

        elif self.control_mode == 'POSITION':
            # Position Control Mode
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

        elif self.control_mode == 'LANDING':
            # Full Offboard Landing State Machine
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.jerk = [float('nan'), float('nan'), float('nan')]
            msg.yawspeed = float('nan')

            if self.land_state == 'SLAM_APPROACH':
                # Fly to 1.5m above rover using SLAM
                if not self.rover_pose_received:
                    # Fallback if no rover pose: hover in place
                    msg.position = [self.drone_pos[0], self.drone_pos[1], self.drone_pos[2]]
                    msg.yaw = self.current_yaw
                else:
                    hover_z = self.rover_pos_ned[2] - 1.5 # 1.5m above rover (NED Down is negative up)
                    msg.position = [self.rover_pos_ned[0], self.rover_pos_ned[1], hover_z]
                    msg.yaw = self.rover_yaw_ned

                    # Check if we are close enough to transition
                    dist_xy = math.sqrt((self.drone_pos[0] - self.rover_pos_ned[0])**2 + (self.drone_pos[1] - self.rover_pos_ned[1])**2)
                    dist_z = abs(self.drone_pos[2] - hover_z)
                    
                    if dist_xy < 0.2 and dist_z < 0.2:
                        self.land_state = 'ARUCO_HOVER'
                        self.get_logger().info("Phase 2: ARUCO_HOVER. Looking for marker...")

            elif self.land_state == 'ARUCO_HOVER':
                # Hold SLAM hover and wait for ArUco
                hover_z = self.rover_pos_ned[2] - 1.5
                msg.position = [self.rover_pos_ned[0], self.rover_pos_ned[1], hover_z]
                msg.yaw = self.rover_yaw_ned

                if self.aruco_visible:
                    self.land_state = 'ARUCO_DESCENT'
                    self.get_logger().info("Phase 3: ARUCO_DESCENT. Marker found, descending...")

            elif self.land_state == 'ARUCO_DESCENT':
                if not self.aruco_visible:
                    # If we lose it when we are very close (< 15cm), it's probably because it's too big for FOV.
                    # Transition to READY_TO_COUPLE blindly.
                    if 0 < self.aruco_rel_pos[2] < 0.15:
                        self.land_state = 'READY_TO_COUPLE'
                        self.get_logger().info("Phase 4: READY_TO_COUPLE (Blind). Marker lost, but <15cm away. Pushing down.")
                    else:
                        # If lost from high up, abort back to hover!
                        self.land_state = 'SLAM_APPROACH'
                        self.get_logger().warn("Marker lost! Aborting back to SLAM Hover.")
                        msg.position = [self.drone_pos[0], self.drone_pos[1], self.drone_pos[2]]
                        msg.yaw = self.current_yaw
                else:
                    # Descend using relative ArUco coordinates
                    # Target X/Y is current drone position + relative offset
                    target_x = self.drone_pos[0] + self.aruco_rel_pos[0]
                    target_y = self.drone_pos[1] + self.aruco_rel_pos[1]
                    
                    # Target Z is slightly below current to force descent (e.g. 0.2m/s descent)
                    target_z = self.drone_pos[2] + 0.01  # Small step down each 50ms (0.2 m/s)
                    
                    # Target Yaw
                    target_yaw = self.current_yaw + self.aruco_rel_yaw

                    msg.position = [target_x, target_y, target_z]
                    msg.yaw = target_yaw

                    # Check if close enough to trigger coupling (e.g. distance to marker < 5cm)
                    if self.aruco_rel_pos[2] < 0.05:
                        self.land_state = 'READY_TO_COUPLE'
                        self.get_logger().info("Phase 4: READY_TO_COUPLE. < 5cm from marker, pushing down gently.")

            elif self.land_state == 'READY_TO_COUPLE':
                if not self.aruco_visible:
                    # Blindly push down and hold current X/Y/Yaw
                    msg.position = [self.drone_pos[0], self.drone_pos[1], self.drone_pos[2] + 0.01]
                    msg.yaw = self.current_yaw
                else:
                    # Keep maintaining X/Y/Yaw over the marker, and push down gently
                    target_x = self.drone_pos[0] + self.aruco_rel_pos[0]
                    target_y = self.drone_pos[1] + self.aruco_rel_pos[1]
                    target_z = self.drone_pos[2] + 0.01 # keep descending slowly
                    target_yaw = self.current_yaw + self.aruco_rel_yaw

                    msg.position = [target_x, target_y, target_z]
                    msg.yaw = target_yaw
                
                # Publish readiness signal for coupling
                success_msg = Bool()
                success_msg.data = True
                self.landing_success_pub.publish(success_msg)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        if self.control_mode == 'POSITION' or self.control_mode == 'LANDING':
            msg.position = True
            msg.velocity = False
        else:
            msg.position = False
            msg.velocity = True
            
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

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