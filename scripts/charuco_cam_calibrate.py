#!/usr/bin/env python3
"""
ChArUco camera intrinsics calibration (standalone).

Default board matches an 8x5 ChArUco grid, 4x4 ArUco markers, 30 mm squares,
22 mm marker edge length (physical units in meters internally).

Usage (offline, recommended):
  python scripts/charuco_cam_calibrate.py --images-dir ./calib_frames

Usage (ROS 2, optional — saves PNGs when detection succeeds):
  python scripts/charuco_cam_calibrate.py --ros-topic /your/camera/image_raw \\
      --ros-save-dir ./calib_frames --ros-target 40

Requires OpenCV *contrib* Python bindings (aruco lives in contrib):
  pip install opencv-contrib-python

ROS 2 mode additionally needs: rclpy, sensor_msgs, cv_bridge (same as your workspace).

This file is intentionally standalone and does not modify other project code.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np


def _dict_id_from_name(name: str) -> int:
    upper = name.strip().upper()
    if not hasattr(cv2.aruco, upper):
        choices = [x for x in dir(cv2.aruco) if x.startswith("DICT_")]
        raise SystemExit(f"Unknown ArUco dictionary {name!r}. Examples: {choices[:8]} ...")
    return int(getattr(cv2.aruco, upper))


def _get_predefined_dictionary(dict_id: int) -> Any:
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _make_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dictionary: Any,
) -> Any:
    size = (int(squares_x), int(squares_y))
    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            return cv2.aruco.CharucoBoard(size, float(square_length_m), float(marker_length_m), dictionary)
        except TypeError:
            # Some builds use (squaresX, squaresY, ...) positional without tuple.
            return cv2.aruco.CharucoBoard(
                int(squares_x),
                int(squares_y),
                float(square_length_m),
                float(marker_length_m),
                dictionary,
            )
    return cv2.aruco.CharucoBoard_create(
        int(squares_x),
        int(squares_y),
        float(square_length_m),
        float(marker_length_m),
        dictionary,
    )


def _default_detector_params() -> Any:
    params = cv2.aruco.DetectorParameters()
    # Slightly more tolerant for consumer / wide-FOV cameras; user can tune in OpenCV if needed.
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def _detect_markers(gray: np.ndarray, dictionary: Any, params: Any) -> Tuple[Any, Any, Any]:
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def _interpolate_charuco(
    marker_corners: Any,
    marker_ids: Any,
    gray: np.ndarray,
    board: Any,
    camera_matrix: Optional[np.ndarray] = None,
    dist_coeffs: Optional[np.ndarray] = None,
) -> Tuple[int, np.ndarray, np.ndarray]:
    if camera_matrix is None:
        camera_matrix = np.eye(3, dtype=np.float64)
    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    return cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
    )


def _calibrate_camera_charuco(
    charuco_corners: Sequence[np.ndarray],
    charuco_ids: Sequence[np.ndarray],
    board: Any,
    image_size: Tuple[int, int],
) -> Tuple[float, np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray]]:
    flags = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)
    return cv2.aruco.calibrateCameraCharuco(
        list(charuco_corners),
        list(charuco_ids),
        board,
        image_size,
        None,
        None,
        flags=flags,
        criteria=criteria,
    )


def _gather_images_from_dir(images_dir: Path) -> List[np.ndarray]:
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}
    paths = sorted([p for p in images_dir.iterdir() if p.suffix.lower() in exts])
    if not paths:
        raise SystemExit(f"No images found in {images_dir}")
    images: List[np.ndarray] = []
    for p in paths:
        im = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if im is None:
            print(f"[warn] skip (unreadable): {p}", file=sys.stderr)
            continue
        images.append(im)
    if len(images) < 3:
        raise SystemExit("Need at least 3 readable images for a stable calibration.")
    return images


def _write_ros_yaml(
    out_path: Path,
    camera_name: str,
    image_width: int,
    image_height: int,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> None:
    """Write a ROS1-style camera_calibration YAML (widely consumed by ROS2 camera_info tools)."""
    k = camera_matrix.reshape(-1).tolist()
    d = dist_coeffs.reshape(-1).tolist()
    # Pad distortion to 5 for plumb_bob if needed
    while len(d) < 5:
        d.append(0.0)
    d = d[:5]

    fx, _, cx, _, fy, cy, _, _, _ = k
    p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    text = f"""image_width: {int(image_width)}
image_height: {int(image_height)}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: {k}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(d)}
  data: {d}
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0., 0., 1., 0., 0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: {p}
"""
    out_path.write_text(text, encoding="utf-8")


def _write_npz(out_path: Path, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, rms: float) -> None:
    np.savez(out_path, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rms=np.array([rms], dtype=np.float64))


def _count_charuco_corners(
    bgr: np.ndarray,
    board: Any,
    dictionary: Any,
    params: Any,
    min_corners: int,
) -> int:
    """Return number of interpolated ChArUco corners, or 0 if unusable."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detect_markers(gray, dictionary, params)
    if ids is None or len(ids) == 0:
        return 0
    ret, char_corners, char_ids = _interpolate_charuco(corners, ids, gray, board)
    if not ret or char_ids is None or char_corners is None:
        return 0
    n = int(len(char_ids))
    return n if n >= int(min_corners) else 0


def _try_ros2_capture(args: argparse.Namespace) -> List[np.ndarray]:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError as e:
        raise SystemExit(
            "ROS 2 capture requested but rclpy/sensor_msgs are not available in this environment.\n"
            f"Import error: {e}"
        ) from e

    try:
        from cv_bridge import CvBridge
    except ImportError as e:
        raise SystemExit(
            "ROS 2 capture needs cv_bridge in the same Python environment.\n" f"Import error: {e}"
        ) from e

    topic: str = args.ros_topic
    save_dir: Optional[Path] = Path(args.ros_save_dir) if args.ros_save_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    square_m = float(args.square_mm) / 1000.0
    marker_m = float(args.marker_mm) / 1000.0
    dict_id = _dict_id_from_name(str(args.dict_name))
    dictionary = _get_predefined_dictionary(dict_id)
    board = _make_charuco_board(
        int(args.squares_x),
        int(args.squares_y),
        square_m,
        marker_m,
        dictionary,
    )
    params = _default_detector_params()
    min_corners = int(args.min_charuco_corners)
    require_charuco = not bool(args.ros_any_frame)

    rclpy.init()
    bridge = CvBridge()
    frames: List[np.ndarray] = []

    class _Cap(Node):
        def __init__(self) -> None:
            super().__init__("charuco_cam_calibrate_capture")
            self._bridge = bridge
            self._count = 0
            self._target = int(args.ros_target)
            self._topic = topic
            self._save_dir = save_dir
            self._board = board
            self._dictionary = dictionary
            self._params = params
            self._min_corners = min_corners
            self._require_charuco = require_charuco

            if "/compressed" in topic or topic.endswith("/compressed"):
                self.create_subscription(
                    CompressedImage,
                    topic,
                    self._on_compressed,
                    qos_profile=qos_profile_sensor_data,
                )
            else:
                self.create_subscription(
                    Image,
                    topic,
                    self._on_raw,
                    qos_profile=qos_profile_sensor_data,
                )

        def _on_raw(self, msg: Image) -> None:
            try:
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as ex:
                self.get_logger().error(f"cv_bridge failed: {ex}")
                return
            self._maybe_accept(bgr)

        def _on_compressed(self, msg: CompressedImage) -> None:
            try:
                bgr = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as ex:
                self.get_logger().error(f"cv_bridge failed: {ex}")
                return
            self._maybe_accept(bgr)

        def _maybe_accept(self, bgr: np.ndarray) -> None:
            if self._count >= self._target:
                return
            if self._require_charuco:
                n = _count_charuco_corners(bgr, self._board, self._dictionary, self._params, self._min_corners)
                if n == 0:
                    return
            frames.append(bgr.copy())
            self._count += 1
            self.get_logger().info(f"Captured frame {self._count}/{self._target} from {self._topic}")
            if self._save_dir is not None:
                out = self._save_dir / f"frame_{self._count:04d}.png"
                cv2.imwrite(str(out), bgr)

    node = _Cap()
    print(f"Listening on {topic!r}; collecting {args.ros_target} frames...")
    if require_charuco:
        print(
            f"ROS capture mode: keeping frames with >= {min_corners} ChArUco corners "
            f"(disable with --ros-any-frame)."
        )
    print("Move the ChArUco board: tilt, distance, and toward image borders for best results.")
    start = time.time()
    while rclpy.ok() and len(frames) < int(args.ros_target):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - start > float(args.ros_timeout_s):
            break
    node.destroy_node()
    rclpy.shutdown()

    if len(frames) < 3:
        raise SystemExit(
            f"Too few ROS frames collected ({len(frames)}). Check the topic name, QoS, and timeout, "
            f"or use --images-dir with saved PNGs."
        )
    return frames


def run_calibration(
    images_bgr: Sequence[np.ndarray],
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dict_name: str,
    min_charuco_corners: int,
    preview_path: Optional[Path],
) -> Tuple[float, np.ndarray, np.ndarray, Tuple[int, int]]:
    dict_id = _dict_id_from_name(dict_name)
    dictionary = _get_predefined_dictionary(dict_id)
    board = _make_charuco_board(squares_x, squares_y, square_length_m, marker_length_m, dictionary)
    params = _default_detector_params()

    h0, w0 = images_bgr[0].shape[:2]
    all_corners: List[np.ndarray] = []
    all_ids: List[np.ndarray] = []
    used = 0

    preview_img: Optional[np.ndarray] = None

    for idx, bgr in enumerate(images_bgr):
        if bgr.shape[0] != h0 or bgr.shape[1] != w0:
            raise SystemExit("All images must have the same resolution for calibration.")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = _detect_markers(gray, dictionary, params)
        if ids is None or len(ids) == 0:
            print(f"[skip] image {idx}: no ArUco markers detected")
            continue

        ret, char_corners, char_ids = _interpolate_charuco(corners, ids, gray, board)
        if not ret or char_ids is None or char_corners is None:
            print(f"[skip] image {idx}: Charuco interpolation failed")
            continue
        n = int(len(char_ids))
        if n < int(min_charuco_corners):
            print(f"[skip] image {idx}: only {n} charuco corners (< {min_charuco_corners})")
            continue

        all_corners.append(char_corners)
        all_ids.append(char_ids)
        used += 1

        if preview_path is not None and preview_img is None:
            vis = bgr.copy()
            if hasattr(cv2.aruco, "drawDetectedCornersCharuco"):
                cv2.aruco.drawDetectedCornersCharuco(vis, char_corners, char_ids)
            else:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            preview_img = vis

    if used < 3:
        raise SystemExit(
            f"Not enough valid detections ({used}). Lower --min-charuco-corners, improve lighting/focus, "
            f"or verify board parameters (--squares-x/y, lengths, --dict)."
        )

    if preview_path is not None and preview_img is not None:
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(preview_path), preview_img)
        print(f"Wrote detection preview: {preview_path}")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = _calibrate_camera_charuco(
        all_corners, all_ids, board, (w0, h0)
    )
    return float(rms), camera_matrix, dist_coeffs, (w0, h0)


def main() -> None:
    p = argparse.ArgumentParser(description="ChArUco intrinsics calibration (standalone).")
    p.add_argument("--images-dir", type=str, default=None, help="Directory of calibration images (png/jpg/...).")
    p.add_argument("--ros-topic", type=str, default=None, help="ROS 2 topic (Image or */compressed).")
    p.add_argument(
        "--ros-save-dir",
        type=str,
        default=None,
        help="If set, save each captured ROS frame as PNG into this directory.",
    )
    p.add_argument("--ros-target", type=int, default=40, help="How many frames to capture from ROS.")
    p.add_argument("--ros-timeout-s", type=float, default=120.0, help="Give up after this many seconds (ROS mode).")
    p.add_argument(
        "--ros-any-frame",
        action="store_true",
        help="ROS mode: capture consecutive frames without requiring ChArUco detection (usually worse for calibration).",
    )

    p.add_argument("--squares-x", type=int, default=8, help="ChArUco squares in X (default 8).")
    p.add_argument("--squares-y", type=int, default=5, help="ChArUco squares in Y (default 5).")
    p.add_argument("--square-mm", type=float, default=30.0, help="Checker square outer size in millimeters.")
    p.add_argument("--marker-mm", type=float, default=22.0, help="Printed ArUco marker outer size in millimeters.")
    p.add_argument("--dict", dest="dict_name", type=str, default="DICT_4X4_50", help="OpenCV predefined dictionary name.")

    p.add_argument("--min-charuco-corners", type=int, default=6, help="Minimum interpolated corners to accept a frame.")
    p.add_argument("--preview-path", type=str, default="", help="If set, write one overlay debug image to this path.")

    p.add_argument("--out-yaml", type=str, default="charuco_camera.yaml", help="Output ROS-style camera YAML path.")
    p.add_argument("--out-npz", type=str, default="charuco_camera.npz", help="Output numpy archive with K, D, rms.")
    p.add_argument("--out-json-meta", type=str, default="", help="Optional JSON path with parameters + RMS.")
    p.add_argument("--camera-name", type=str, default="calibrated_camera", help="camera_name field in YAML.")

    args = p.parse_args()

    if not args.images_dir and not args.ros_topic:
        p.error("Provide either --images-dir or --ros-topic.")
    if args.images_dir and args.ros_topic:
        p.error("Choose one input mode: --images-dir OR --ros-topic (not both).")

    square_m = float(args.square_mm) / 1000.0
    marker_m = float(args.marker_mm) / 1000.0
    if marker_m >= square_m:
        raise SystemExit("--marker-mm must be smaller than --square-mm (marker fits inside the square).")

    if args.images_dir:
        images = _gather_images_from_dir(Path(args.images_dir))
    else:
        images = _try_ros2_capture(args)

    preview = Path(args.preview_path) if str(args.preview_path).strip() else None

    rms, k, d, (w, h) = run_calibration(
        images,
        squares_x=int(args.squares_x),
        squares_y=int(args.squares_y),
        square_length_m=square_m,
        marker_length_m=marker_m,
        dict_name=str(args.dict_name),
        min_charuco_corners=int(args.min_charuco_corners),
        preview_path=preview,
    )

    out_yaml = Path(args.out_yaml)
    out_npz = Path(args.out_npz)
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    out_npz.parent.mkdir(parents=True, exist_ok=True)

    _write_ros_yaml(out_yaml, camera_name=str(args.camera_name), image_width=w, image_height=h, camera_matrix=k, dist_coeffs=d)
    _write_npz(out_npz, camera_matrix=k, dist_coeffs=d, rms=rms)

    if str(args.out_json_meta).strip():
        meta = {
            "rms": rms,
            "image_width": w,
            "image_height": h,
            "squares_x": int(args.squares_x),
            "squares_y": int(args.squares_y),
            "square_mm": float(args.square_mm),
            "marker_mm": float(args.marker_mm),
            "dict": str(args.dict_name),
            "opencv_version": cv2.__version__,
        }
        Path(args.out_json_meta).write_text(json.dumps(meta, indent=2), encoding="utf-8")

    print(f"Calibration RMS reprojection error: {rms:.4f} px")
    print(f"Wrote: {out_yaml}")
    print(f"Wrote: {out_npz}")


if __name__ == "__main__":
    main()
