#!/usr/bin/env python3
"""
ChArUco camera calibration for the downward camera.

Board defaults match an 8×5 ChArUco grid: 30 mm square spacing, 22 mm marker size,
DICT_4X4_50 (same family as typical printed ChArUco boards).

Subscribes to an image topic (default /downward_camera/image_raw), collects captures
with [c], runs calibration and writes YAML with [s], quits with [q].

Output YAML is compatible with pacj.aruco_detector_node (camera_matrix.data,
distortion_coefficients.data).
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


def _opencv_preimport_env() -> None:
    # Reduce odds of GUI backends initializing in headless/SSH contexts.
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    # Helpful on some ARM setups when OpenCV pulls in GL stacks unexpectedly.
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def _opencv_postimport_init() -> None:
    import cv2

    # Avoid rare crashes / oversubscription on embedded CPUs.
    try:
        cv2.setUseOptimized(False)
    except Exception:
        pass
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass


_opencv_preimport_env()
import cv2  # noqa: E402

_opencv_postimport_init()


class CharucoCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_calibration")

        self.declare_parameter("squares_x", 8)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.030)
        self.declare_parameter("marker_length", 0.022)
        self.declare_parameter("aruco_dict", 0)  # DICT_4X4_50
        self.declare_parameter("image_topic", "/downward_camera/image_raw")
        self.declare_parameter("output_path", "~/camera_calibration.yaml")
        self.declare_parameter("min_captures", 15)
        self.declare_parameter("headless", False)

        sx = int(self.get_parameter("squares_x").value)
        sy = int(self.get_parameter("squares_y").value)
        square_length = float(self.get_parameter("square_length").value)
        marker_length = float(self.get_parameter("marker_length").value)
        dict_id = int(self.get_parameter("aruco_dict").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self._output_path = os.path.expanduser(str(self.get_parameter("output_path").value))
        self._min_captures = int(self.get_parameter("min_captures").value)
        headless_param = bool(self.get_parameter("headless").value)

        # OpenCV highgui is fragile in headless/SSH sessions and can segfault.
        # If DISPLAY isn't set, force headless regardless of the param.
        self._headless = headless_param or (os.environ.get("DISPLAY") in (None, ""))

        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_dict = aruco_dict
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._board = cv2.aruco.CharucoBoard(
            (sx, sy),
            square_length,
            marker_length,
            aruco_dict,
        )

        self._all_corners: List[np.ndarray] = []
        self._all_ids: List[np.ndarray] = []
        self._image_size: Optional[Tuple[int, int]] = None

        self._bridge = CvBridge()
        self._latest_frame: Optional[Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]] = None
        self._latest_annotated: Optional[np.ndarray] = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        if not self._headless:
            self.create_timer(0.1, self._display_and_keys)

        # Headless control via ROS services (works over SSH).
        self.create_service(Trigger, "capture", self._srv_capture)
        self.create_service(Trigger, "calibrate", self._srv_calibrate)

        mode_msg = (
            "  [c] capture  [s] calibrate & save YAML  [q] quit"
            if not self._headless
            else (
                "  headless=True (no OpenCV window)\n"
                "  services:\n"
                "    ros2 service call /charuco_calibration/capture std_srvs/srv/Trigger {}\n"
                "    ros2 service call /charuco_calibration/calibrate std_srvs/srv/Trigger {}"
            )
        )
        self.get_logger().info(
            f"ChArUco calibration ({sx}×{sy} squares, {square_length * 1000:.0f} mm square, "
            f"{marker_length * 1000:.0f} mm markers, dict_id={dict_id})\n"
            f"  image_topic={image_topic}\n"
            f"  output_path={self._output_path}\n"
            + mode_msg
        )

    def _image_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use the older, widely-available OpenCV ArUco/ChArUco API.
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params
        )

        charuco_corners = None
        charuco_ids = None
        if ids is not None and len(ids) > 0:
            ich = cv2.aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=self._board,
            )
            # OpenCV versions differ on return arity; be defensive.
            if isinstance(ich, tuple) and len(ich) >= 3:
                charuco_corners = ich[1]
                charuco_ids = ich[2]
            else:
                charuco_corners = None
                charuco_ids = None
        annotated = frame.copy()

        if charuco_ids is not None and len(charuco_ids) >= 4:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
            n = len(charuco_ids)
            cv2.putText(
                annotated,
                f"Detected {n} corners",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                annotated,
                "No ChArUco (need >= 4 corners)",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        cv2.putText(
            annotated,
            f"Captures: {len(self._all_ids)}/{self._min_captures}  [c] [s] [q]",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 200, 0),
            2,
        )

        self._latest_frame = (frame, charuco_corners, charuco_ids)
        self._latest_annotated = annotated

    def _display_and_keys(self) -> None:
        if self._latest_annotated is None:
            return
        try:
            cv2.imshow("ChArUco calibration (pacj)", self._latest_annotated)
            key = cv2.waitKey(1) & 0xFF
        except Exception as e:
            # If highgui fails at runtime (common over SSH), switch to headless.
            self.get_logger().error(f"OpenCV GUI failed ({e}); switching to headless mode.")
            self._headless = True
            return

        if key == ord("c"):
            self._capture()
        elif key == ord("s"):
            self._calibrate_and_save()
        elif key == ord("q"):
            self.get_logger().info("Quit.")
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def _srv_capture(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        before = len(self._all_ids)
        self._capture()
        after = len(self._all_ids)
        if after > before:
            res.success = True
            res.message = f"Captured frame #{after}."
        else:
            res.success = False
            res.message = "Capture skipped (no frame yet, or not enough corners)."
        return res

    def _srv_calibrate(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        try:
            self._calibrate_and_save()
            # _calibrate_and_save() logs details; treat missing captures as failure.
            if len(self._all_ids) < self._min_captures:
                res.success = False
                res.message = f"Need at least {self._min_captures} captures."
            else:
                res.success = True
                res.message = f"Saved calibration to {self._output_path}"
        except Exception as e:
            res.success = False
            res.message = f"Calibration failed: {e}"
        return res
    def _capture(self) -> None:
        if self._latest_frame is None:
            return
        _frame, charuco_corners, charuco_ids = self._latest_frame
        if charuco_ids is None or len(charuco_ids) < 4:
            self.get_logger().warn("Not enough ChArUco corners — skip capture.")
            return
        self._all_corners.append(charuco_corners)
        self._all_ids.append(charuco_ids)
        n = len(self._all_ids)
        self.get_logger().info(f"Captured frame #{n} ({len(charuco_ids)} corners)")

    def _calibrate_and_save(self) -> None:
        n = len(self._all_ids)
        if n < self._min_captures:
            self.get_logger().warn(
                f"Only {n} captures — need at least {self._min_captures}. Press [c] on varied poses."
            )
            return
        if self._image_size is None:
            self.get_logger().error("No image size — did not receive any frames?")
            return

        self.get_logger().info(f"Calibrating with {n} frames …")
        ret, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.aruco.calibrateCameraCharuco(
            self._all_corners,
            self._all_ids,
            self._board,
            self._image_size,
            None,
            None,
        )

        self.get_logger().info(f"RMS reprojection error: {ret:.4f} px")
        if ret > 1.0:
            self.get_logger().warn("RMS > 1.0 px — try more tilt/distance/angle diversity.")

        dist_flat = dist_coeffs.flatten()
        data = {
            "image_width": self._image_size[0],
            "image_height": self._image_size[1],
            "camera_name": "downward_camera",
            "camera_matrix": {"rows": 3, "cols": 3, "data": camera_matrix.flatten().tolist()},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(dist_flat.size),
                "data": dist_flat.tolist(),
            },
            "rms_reprojection_error": float(ret),
        }

        out_dir = os.path.dirname(self._output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self._output_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.get_logger().info(f"Saved calibration → {self._output_path}")
        self.get_logger().info(
            "Use with aruco_detector:\n"
            f"  ros2 run pacj aruco_detector --ros-args "
            f"-p camera_calibration_file:={self._output_path}\n"
            "Or add camera_calibration_file to drone.launch.py aruco_detector_node parameters."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CharucoCalibrationNode] = None
    try:
        node = CharucoCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None and (not getattr(node, "_headless", True)):
                cv2.destroyAllWindows()
        except Exception:
            pass
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
