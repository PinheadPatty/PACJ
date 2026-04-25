#!/usr/bin/env python3
"""
ArUco marker pose estimation from the downward camera stream.

Subscribes (defaults):
  /downward_camera/image_raw   (sensor_msgs/Image)
  /downward_camera/camera_info (sensor_msgs/CameraInfo)

Publishes (defaults):
  /aruco/pose        (geometry_msgs/PoseStamped) marker pose in the camera optical frame
  /aruco/image_debug (sensor_msgs/Image)         annotated image (optional)
"""

from __future__ import annotations

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
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image


def _rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = float((R[2, 1] - R[1, 2]) * s)
        y = float((R[0, 2] - R[2, 0]) * s)
        z = float((R[1, 0] - R[0, 1]) * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2]))
        w = float((R[2, 1] - R[1, 2]) / s)
        x = 0.25 * s
        y = float((R[0, 1] + R[1, 0]) / s)
        z = float((R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2]))
        w = float((R[0, 2] - R[2, 0]) / s)
        x = float((R[0, 1] + R[1, 0]) / s)
        y = 0.25 * s
        z = float((R[1, 2] + R[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1]))
        w = float((R[1, 0] - R[0, 1]) / s)
        x = float((R[0, 2] + R[2, 0]) / s)
        y = float((R[1, 2] + R[2, 1]) / s)
        z = 0.25 * s
    return x, y, z, w


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

        self.declare_parameter("marker_size", 0.046)
        self.declare_parameter("aruco_dict", 0)  # DICT_4X4_50
        self.declare_parameter("target_marker_id", -1)  # -1 = any
        self.declare_parameter("camera_calibration_file", "")
        self.declare_parameter("image_topic", "/downward_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/downward_camera/camera_info")
        self.declare_parameter("pose_topic", "/aruco/pose")
        self.declare_parameter("debug_image_topic", "/aruco/image_debug")
        self.declare_parameter("publish_debug_image", True)

        marker_size = float(self.get_parameter("marker_size").value)
        dict_id = int(self.get_parameter("aruco_dict").value)
        self._target_id = int(self.get_parameter("target_marker_id").value)
        calib_file = str(self.get_parameter("camera_calibration_file").value)

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)

        self._marker_size = marker_size

        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.adaptiveThreshWinSizeMin = 3
        detector_params.adaptiveThreshWinSizeMax = 23
        detector_params.adaptiveThreshWinSizeStep = 4
        detector_params.minMarkerPerimeterRate = 0.03
        detector_params.maxMarkerPerimeterRate = 0.5
        detector_params.polygonalApproxAccuracyRate = 0.05
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

        half = self._marker_size / 2.0
        self._obj_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float64,
        )

        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_info_received = False

        if calib_file:
            K, D = _load_calibration_yaml(calib_file)
            if K is not None:
                self._camera_matrix = K
                self._dist_coeffs = D
                self._camera_info_received = True
                self.get_logger().info(f"Loaded calibration from {calib_file}")
            else:
                self.get_logger().warn(
                    f"Failed to load {calib_file}; will use {camera_info_topic} instead."
                )

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._debug_pub = self.create_publisher(Image, debug_topic, 1) if self._publish_debug else None

        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, sensor_qos)
        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)

        self.get_logger().info(
            "Aruco detector online.\n"
            f"  image_topic={image_topic}\n"
            f"  camera_info_topic={camera_info_topic}\n"
            f"  marker_size={self._marker_size:.3f} m\n"
            f"  target_id={'any' if self._target_id < 0 else self._target_id}\n"
            f"  pose_topic={pose_topic}\n"
            f"  debug_topic={'(disabled)' if not self._publish_debug else debug_topic}"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._camera_info_received:
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        self._camera_info_received = True

    def _image_cb(self, msg: Image) -> None:
        if not self._camera_info_received or self._camera_matrix is None or self._dist_coeffs is None:
            self.get_logger().warn("Waiting for camera intrinsics…", throttle_duration_sec=5.0)
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = self._detector.detectMarkers(gray)

        annotated = frame.copy()
        if ids is None or len(ids) == 0:
            self._publish_debug_image(annotated, msg.header)
            return

        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if self._target_id >= 0 and int(marker_id) != self._target_id:
                continue

            marker_corners = corners[i].reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._obj_points,
                marker_corners,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            cv2.drawFrameAxes(
                annotated,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec,
                self._marker_size * 0.5,
            )

            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = _rotation_matrix_to_quaternion(R)

            pose = PoseStamped()
            pose.header = msg.header  # same stamp & frame_id as the image stream
            pose.pose.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
            pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self._pose_pub.publish(pose)

        self._publish_debug_image(annotated, msg.header)

    def _publish_debug_image(self, frame: np.ndarray, header) -> None:
        if not self._publish_debug or self._debug_pub is None:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header = header
        self._debug_pub.publish(msg)


def main(args=None) -> None:
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

