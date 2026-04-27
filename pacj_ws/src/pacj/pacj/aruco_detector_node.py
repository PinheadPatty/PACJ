#!/usr/bin/env python3
"""
ArUco marker pose estimation - Drone-Relative Version.
Converts Camera coordinates to Drone Body coordinates:
  X = Forward, Y = Left, Z = Distance (Altitude).
"""

import math
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image, CompressedImage

def default_camera_calibration_yaml():
    try:
        base_path = os.path.join(get_package_share_directory('pacj'), 'calibration')
        return os.path.join(base_path, filename)
    except Exception:
        return ""

def _load_calibration_yaml(path: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(1, -1)
        return K, D
    except Exception:
        return None, None

class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")

        # --- Parameters ---
        self.declare_parameter("marker_size", 0.046)
        self.declare_parameter("aruco_dict", 0)  # DICT_4X4_50
        self.declare_parameter("camera_calibration_file", default_camera_calibration_yaml())
        self.declare_parameter("image_topic", "/drone/downward_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/drone/downward_camera/camera_info")
        self.declare_parameter("relative_pose_topic", "/drone/aruco/relative_pose")

        self._marker_size = float(self.get_parameter("marker_size").value)
        calib_file = str(self.get_parameter("camera_calibration_file").value)
        
        # --- Legacy ArUco Setup (Stable for 4.6.0) ---
        dict_id = int(self.get_parameter("aruco_dict").value)
        self._aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        self._detector_params = cv2.aruco.DetectorParameters_create()
        
        # Define 3D points of the marker
        half = self._marker_size / 2.0
        self._obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float64)

        self._camera_matrix = None
        self._dist_coeffs = None
        self._camera_info_received = False
        self._bridge = CvBridge()

        # Load Calibration
        K, D = _load_calibration_yaml(calib_file)
        if K is not None:
            self._camera_matrix, self._dist_coeffs = K, D
            self._camera_info_received = True
            self.get_logger().info(f"Loaded {calib_file}")

        # --- Publishers & Subscribers ---
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        # Main output: Relative X, Y, Z
        self._rel_pub = self.create_publisher(Vector3, self.get_parameter("relative_pose_topic").value, 10)
        
        # Debug streams
        self._debug_pub = self.create_publisher(Image, "/drone/aruco/image_debug", 1)
        self._compressed_pub = self.create_publisher(CompressedImage, "/drone/aruco/image_debug/compressed", 1)

        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info_cb, sensor_qos)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._image_cb, sensor_qos)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._camera_info_received: return
        self._camera_matrix = np.array(msg.k).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d).reshape(1, -1)
        self._camera_info_received = True

    def _image_cb(self, msg: Image) -> None:
        if not self._camera_info_received: return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=self._detector_params)

        annotated = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            
            for i, marker_id in enumerate(ids.flatten()):
                ok, rvec, tvec = cv2.solvePnP(
                    self._obj_points, corners[i].astype(np.float64),
                    self._camera_matrix, self._dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if ok:
                    cv2.drawFrameAxes(annotated, self._camera_matrix, self._dist_coeffs, rvec, tvec, self._marker_size * 0.5)
                    
                    # --- THE FRAME TRANSFORMATION ---
                    # Camera (OpenCV) coords: X=right, Y=down, Z=forward
                    # Drone (Body) coords: X=forward, Y=left, Z=up/altitude
                    
                    # For a downward facing camera:
                    # The marker's Y in the camera is actually how far "Forward" it is.
                    # The marker's X in the camera is actually how far "Right" it is (so we negate for Left).
                    # The marker's Z in the camera is the distance to the ground.
                    
                    fwd = float(tvec[1])    # Camera Y -> Drone Forward
                    left = -float(tvec[0])  # Camera X -> Drone Left
                    dist = float(tvec[2])   # Camera Z -> Distance

                    # Publish Relative Position
                    rel_msg = Vector3(x=fwd, y=left, z=dist)
                    self._rel_pub.publish(rel_msg)

                    self.get_logger().info(
                        f"ID {marker_id} -> FWD: {fwd:.2f}m, LFT: {left:.2f}m, DIST: {dist:.2f}m", 
                        throttle_duration_sec=0.5
                    )

        self._publish_debug(annotated, msg.header)

    def _publish_debug(self, frame: np.ndarray, header) -> None:
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
        self._compressed_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()