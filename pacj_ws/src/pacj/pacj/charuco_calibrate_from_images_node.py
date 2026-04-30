#!/usr/bin/env python3
"""
Offline ChArUco camera calibration from saved images.
Fixed for OpenCV 4.6 (Jazzy/Ubuntu 24.04 ARM) stability.
"""

import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
import cv2
import rclpy
from rclpy.node import Node

def package_calibration_dir() -> Path:
    """Finds calibration images, prioritizing Source folder then Install folder."""
    # 1. Check for manual environment override
    env = os.environ.get("PACJ_CALIB_DIR", "").strip()
    if env:
        return Path(os.path.expanduser(env))

    # 2. Try to find the Source directory (useful for dev)
    # This assumes your workspace is /home/pacj/PACJ/pacj_ws/
    src_path = Path.home() / "PACJ" / "pacj_ws" / "src" / "pacj" / "calibration"
    if src_path.is_dir():
        return src_path

    # 3. Fallback to the ROS-installed share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("pacj")) / "calibration"
        return p
    except Exception:
        pass

    # 4. Ultimate fallback: current working directory
    return Path.cwd() / "calibration"

def _list_image_paths(directory: str) -> List[str]:
    directory = os.path.expanduser(directory)
    if not os.path.exists(directory):
        return []
    exts = {".png", ".jpg", ".jpeg", ".PNG", ".JPG", ".JPEG"}
    return [os.path.join(directory, n) for n in sorted(os.listdir(directory))
            if os.path.isfile(os.path.join(directory, n)) and os.path.splitext(n)[1] in exts]

class CharucoCalibrateFromImagesNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_calibrate_from_images")

        self.declare_parameter("image_directory", str(package_calibration_dir()))
        self.declare_parameter("output_yaml", str(package_calibration_dir() / "camera_calibration.yaml"))
        self.declare_parameter("squares_x", 8)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.029)
        self.declare_parameter("marker_length", 0.021)
        self.declare_parameter("aruco_dict", cv2.aruco.DICT_4X4_50)
        self.declare_parameter("min_valid_images", 10)
        self.declare_parameter("min_charuco_corners", 4)

        self._image_dir = self.get_parameter("image_directory").value
        self._output_yaml = self.get_parameter("output_yaml").value
        self._sx = self.get_parameter("squares_x").value
        self._sy = self.get_parameter("squares_y").value
        self._square_length = self.get_parameter("square_length").value
        self._marker_length = self.get_parameter("marker_length").value
        self._dict_id = self.get_parameter("aruco_dict").value
        self._min_valid = self.get_parameter("min_valid_images").value
        self._min_corners = self.get_parameter("min_charuco_corners").value

        # Use the 4.6 compatible Dictionary_get
        self._aruco_dict = cv2.aruco.Dictionary_get(self._dict_id)
        
        # USE THE _create METHOD - This is vital for OpenCV 4.6 to avoid Segfaults
        self._board = cv2.aruco.CharucoBoard_create(
            self._sx, self._sy, 
            self._square_length, 
            self._marker_length, 
            self._aruco_dict
        )
        
        self._detector_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info(f"Initialized ChArUco {self._sx}x{self._sy} (OpenCV {cv2.__version__})")

    def _interpolate_charuco(self, gray: np.ndarray, bgr_debug: np.ndarray, filename: str) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        # 1. Detect the ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._detector_params
        )

        if ids is not None:
            # Draw detected markers (Yellow boxes)
            cv2.aruco.drawDetectedMarkers(bgr_debug, corners, ids)
            
            # 2. Interpolate the ChArUco corners
            res, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self._board
            )

            if charuco_corners is not None and len(charuco_corners) > 0:
                # Draw the interpolation (Green dots and ID numbers)
                cv2.aruco.drawDetectedCornersCharuco(bgr_debug, charuco_corners, charuco_ids, (0, 255, 0))
                
                # Save the debug image to a 'debug' subfolder
                debug_dir = Path(self._image_dir) / "debug"
                debug_dir.mkdir(exist_ok=True)
                cv2.imwrite(str(debug_dir / f"debug_{filename}"), bgr_debug)
                
                if res > self._min_corners:
                    return charuco_corners, charuco_ids
        
        return None, None

    def run_calibration(self):
        paths = _list_image_paths(self._image_dir)
        if not paths:
            raise RuntimeError(f"No images found in {self._image_dir}")

        all_corners, all_ids = [], []
        image_size = None
        used, skipped = 0, 0

        self.get_logger().info(f"Processing {len(paths)} images for debug and calibration...")

        for path in paths:
            filename = os.path.basename(path)
            bgr = cv2.imread(path)
            if bgr is None:
                skipped += 1
                continue
            
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = gray.shape[::-1]

            # Pass the BGR image to draw on it
            c_corners, c_ids = self._interpolate_charuco(gray, bgr.copy(), filename)
            
            if c_ids is not None:
                all_corners.append(c_corners)
                all_ids.append(c_ids)
                used += 1
            else:
                skipped += 1

        if used < self._min_valid:
            self.get_logger().error(f"Only {used} images passed. Check the 'debug' folder to see why!")
            raise RuntimeError(f"Insufficient images: {used}/{self._min_valid} required.")

        self.get_logger().info(f"Running calibration on {used} images...")

        ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=self._board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )

        self.get_logger().info(f"Calibration successful! RMS: {ret:.4f}")
        self._save_yaml(mtx, dist, image_size, ret)

    def _save_yaml(self, mtx, dist, size, rms):
        data = {
            "image_width": size[0],
            "image_height": size[1],
            "camera_matrix": {"rows": 3, "cols": 3, "data": mtx.flatten().tolist()},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": dist.size, "data": dist.flatten().tolist()},
            "rms": float(rms)
        }
        os.makedirs(os.path.dirname(self._output_yaml), exist_ok=True)
        with open(self._output_yaml, "w") as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Saved to {self._output_yaml}")

def main(args=None):
    rclpy.init(args=args)
    node = CharucoCalibrateFromImagesNode()
    try:
        node.run_calibration()
    except Exception as e:
        node.get_logger().error(f"Calibration failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()