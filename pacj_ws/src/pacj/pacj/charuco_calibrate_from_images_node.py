#!/usr/bin/env python3
"""
Offline ChArUco camera calibration from saved images.

Reads *.png / *.jpg / *.jpeg from image_directory, detects ChArUco corners on each
image, runs calibrateCameraCharuco, and writes YAML compatible with
pacj.aruco_detector_node (camera_matrix.data, distortion_coefficients.data).

Board defaults: 8×5 squares, 30 mm square, 22 mm markers, DICT_4X4_50.

Default ``image_directory`` / ``output_yaml`` use the package ``calibration/``
folder (shared helpers in ``charuco_capture_node``). Override with params or ``PACJ_CALIB_DIR``.

Service:
  ~/calibrate (std_srvs/Trigger) — run calibration and write output_yaml.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

from pacj.charuco_capture_node import default_camera_calibration_yaml, package_calibration_dir


def _opencv_preimport_env() -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


def _opencv_postimport_init() -> None:
    import cv2

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


def _list_image_paths(directory: str) -> List[str]:
    directory = os.path.expanduser(directory)
    exts = {".png", ".jpg", ".jpeg", ".PNG", ".JPG", ".JPEG"}
    names = []
    for n in sorted(os.listdir(directory)):
        path = os.path.join(directory, n)
        if os.path.isfile(path) and os.path.splitext(n)[1] in exts:
            names.append(path)
    return names


def _interpolate_charuco(
    gray: np.ndarray,
    board,
    aruco_dict,
    detector_params,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    corners, ids, _rej = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)
    if ids is None or len(ids) == 0:
        return None, None
    ich = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=gray,
        board=board,
    )
    if isinstance(ich, tuple) and len(ich) >= 3:
        return ich[1], ich[2]
    return None, None


class CharucoCalibrateFromImagesNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_calibrate_from_images")

        self.declare_parameter("image_directory", str(package_calibration_dir()))
        self.declare_parameter("output_yaml", default_camera_calibration_yaml())
        self.declare_parameter("squares_x", 8)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.030)
        self.declare_parameter("marker_length", 0.022)
        self.declare_parameter("aruco_dict", 0)
        self.declare_parameter("min_valid_images", 15)
        self.declare_parameter("min_charuco_corners", 4)

        self._image_dir = os.path.expanduser(str(self.get_parameter("image_directory").value))
        self._output_yaml = os.path.expanduser(str(self.get_parameter("output_yaml").value))
        self._sx = int(self.get_parameter("squares_x").value)
        self._sy = int(self.get_parameter("squares_y").value)
        self._square_length = float(self.get_parameter("square_length").value)
        self._marker_length = float(self.get_parameter("marker_length").value)
        self._dict_id = int(self.get_parameter("aruco_dict").value)
        self._min_valid = int(self.get_parameter("min_valid_images").value)
        self._min_corners = int(self.get_parameter("min_charuco_corners").value)

        aruco_dict = cv2.aruco.getPredefinedDictionary(self._dict_id)
        self._board = cv2.aruco.CharucoBoard(
            (self._sx, self._sy),
            self._square_length,
            self._marker_length,
            aruco_dict,
        )
        self._aruco_dict = aruco_dict
        self._detector_params = cv2.aruco.DetectorParameters()

        self.create_service(Trigger, "calibrate", self._srv_calibrate)

        self.get_logger().info(
            "ChArUco offline calibrator\n"
            f"  image_directory={self._image_dir}\n"
            f"  output_yaml={self._output_yaml}\n"
            f"  board={self._sx}×{self._sy}, square={self._square_length} m, marker={self._marker_length} m\n"
            f"  min_valid_images={self._min_valid}\n"
            "  ros2 service call /charuco_calibrate_from_images/calibrate std_srvs/srv/Trigger {}"
        )

    def _srv_calibrate(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        try:
            out = self._run_calibration()
            res.success = True
            res.message = out
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
        return res

    def _run_calibration(self) -> str:
        paths = _list_image_paths(self._image_dir)
        if not paths:
            raise RuntimeError(f"No images found in {self._image_dir}")

        all_corners: List[np.ndarray] = []
        all_ids: List[np.ndarray] = []
        image_size: Optional[Tuple[int, int]] = None
        used = 0
        skipped = 0

        for path in paths:
            bgr = cv2.imread(path, cv2.IMREAD_COLOR)
            if bgr is None:
                skipped += 1
                self.get_logger().warn(f"Could not read image: {path}")
                continue
            h, w = bgr.shape[:2]
            if image_size is None:
                image_size = (w, h)
            elif (w, h) != image_size:
                raise RuntimeError(
                    f"Image size mismatch: {path} is {(w, h)}, expected {image_size}. "
                    "Use the same camera resolution for every capture."
                )

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            c_corners, c_ids = _interpolate_charuco(gray, self._board, self._aruco_dict, self._detector_params)
            if c_ids is None or len(c_ids) < self._min_corners:
                skipped += 1
                continue

            all_corners.append(c_corners)
            all_ids.append(c_ids)
            used += 1

        if used < self._min_valid:
            raise RuntimeError(
                f"Only {used} images with >= {self._min_corners} ChArUco corners "
                f"(need {self._min_valid}). Skipped {skipped} images."
            )
        if image_size is None:
            raise RuntimeError("Internal error: no image size")

        self.get_logger().info(f"Calibrating with {used} valid images (skipped {skipped}) …")
        ret, camera_matrix, dist_coeffs, _rv, _tv = cv2.aruco.calibrateCameraCharuco(
            all_corners,
            all_ids,
            self._board,
            image_size,
            None,
            None,
        )

        self.get_logger().info(f"RMS reprojection error: {ret:.4f} px")

        dist_flat = dist_coeffs.flatten()
        data = {
            "image_width": image_size[0],
            "image_height": image_size[1],
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

        out_path = self._output_yaml
        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.get_logger().info(f"Saved calibration → {out_path}")
        return f"Used {used} images, wrote {out_path}, RMS={ret:.4f}px"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoCalibrateFromImagesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
