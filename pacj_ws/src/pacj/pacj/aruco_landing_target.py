#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import LandingTargetPose, VehicleStatus

import cv2
from cv_bridge import CvBridge
import numpy as np

class ArucoLandingTarget(Node):
    def __init__(self):
        super().__init__('aruco_landing_target')

        # Configure QoS profile for PX4 DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.target_pub = self.create_publisher(
            LandingTargetPose, '/fmu/in/landing_target_pose', qos_profile)

        self.bridge = CvBridge()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.marker_size = self.declare_parameter('marker_size', 0.15).value  # 15 cm default
        
        # We use a standard dictionary, e.g. 4x4
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
            
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)

        self.active_landing_mode = False

        self.get_logger().info("ArUco Landing Target Node Initialized.")

    def vehicle_status_cb(self, msg):
        # Check if drone is in a mode that uses precision landing
        # NAVIGATION_STATE_AUTO_PRECLAND (20) = Explicit precision land mode
        # NAVIGATION_STATE_AUTO_RTL (5) = Return to Launch (which we configured to use precision landing)
        is_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND or 
                     msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL)
                     
        if is_active != self.active_landing_mode:
            if is_active:
                self.get_logger().info("Precision landing mode ACTIVATED. Starting camera processing.")
            else:
                self.get_logger().info("Precision landing mode DEACTIVATED. Suspending camera processing.")
            self.active_landing_mode = is_active

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info received.")

    def image_cb(self, msg):
        if not self.active_landing_mode:
            return

        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            # 3D points of the marker corners
            marker_points = np.array([
                [-self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            _, rvec, tvec = cv2.solvePnP(marker_points, corners[0][0], self.camera_matrix, self.dist_coeffs)
            tvec = tvec.flatten()
            
            # OpenCV Camera Frame: X right, Y down, Z forward
            # Assuming downward-facing camera on drone:
            # Camera X = Drone Right (East)
            # Camera Y = Drone Backward (-North)
            # Camera Z = Drone Down (Down)
            
            # PX4 NED Frame: X North, Y East, Z Down
            x_rel = -tvec[1]
            y_rel = tvec[0]
            z_rel = tvec[2]

            target_msg = LandingTargetPose()
            target_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            target_msg.is_static = True
            
            target_msg.rel_pos_valid = True
            target_msg.x_rel = float(x_rel)
            target_msg.y_rel = float(y_rel)
            target_msg.z_rel = float(z_rel)
            
            # We are not estimating velocity here
            target_msg.rel_vel_valid = False
            
            self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingTarget()
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