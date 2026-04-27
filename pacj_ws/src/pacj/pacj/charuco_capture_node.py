#!/usr/bin/env python3
"""
Save camera frames to disk for offline ChArUco calibration.

On start: clears prior ``{file_prefix}#####.{ext}`` in ``save_directory``, subscribes
to ``image_topic``, then saves ``auto_save_count`` frames on a fixed timer (no service).

Default ``save_directory`` is under the ``pacj`` package ``calibration/`` folder
(``package_calibration_dir()`` in this module). Override with param or ``PACJ_CALIB_DIR``.

Parameters:
  auto_save_count (int, default 50) — number of frames to save
  auto_save_interval_s (float, default 0.4) — seconds between save attempts
"""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Optional

# Keep OpenCV from pulling GUI stacks when only imgcodecs is needed.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.timer import Timer

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def package_calibration_dir() -> Path:
    """Directory for ChArUco captures and camera calibration YAML."""
    env = os.environ.get("PACJ_CALIB_DIR", "").strip()
    if env:
        p = Path(os.path.expanduser(env))
        p.mkdir(parents=True, exist_ok=True)
        return p

    root = Path(__file__).resolve().parent.parent
    if (root / "package.xml").is_file():
        p = root / "calibration"
        p.mkdir(parents=True, exist_ok=True)
        return p

    from ament_index_python.packages import get_package_share_directory

    p = Path(get_package_share_directory("pacj")) / "calibration"
    p.mkdir(parents=True, exist_ok=True)
    return p


def default_camera_calibration_yaml() -> str:
    return str(package_calibration_dir() / "camera_calibration.yaml")


class CharucoCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_capture")

        self.declare_parameter("image_topic", "/drone/downward_camera/image_raw")
        self.declare_parameter("save_directory", str(package_calibration_dir()))
        self.declare_parameter("file_prefix", "frame_")
        self.declare_parameter("file_extension", "png")  # png or jpg
        self.declare_parameter("auto_save_count", 20)
        self.declare_parameter("auto_save_interval_s", 2)

        image_topic = str(self.get_parameter("image_topic").value)
        self._save_dir = os.path.expanduser(str(self.get_parameter("save_directory").value))
        self._prefix = str(self.get_parameter("file_prefix").value)
        ext = str(self.get_parameter("file_extension").value).lower().lstrip(".")
        if ext not in ("png", "jpg", "jpeg"):
            raise ValueError("file_extension must be png, jpg, or jpeg")
        if ext == "jpeg":
            ext = "jpg"
        self._ext = ext

        auto_n = int(self.get_parameter("auto_save_count").value)
        auto_dt = float(self.get_parameter("auto_save_interval_s").value)
        if auto_n < 1:
            raise ValueError("auto_save_count must be >= 1")
        if auto_dt <= 0.0:
            raise ValueError("auto_save_interval_s must be > 0")

        os.makedirs(self._save_dir, exist_ok=True)
        self._startup_removed = self._clear_existing_capture_files()
        self._next_index = 0
        self._bridge = CvBridge()
        self._latest_bgr: Optional[np.ndarray] = None

        self._auto_timer: Optional[Timer] = None
        self._auto_remaining = auto_n
        self._auto_target = auto_n

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self._auto_timer = self.create_timer(auto_dt, self._auto_tick)

        self.get_logger().info(
            f"ChArUco capture: will save {auto_n} frames every {auto_dt:.2f}s\n"
            f"  image_topic={image_topic}\n"
            f"  save_directory={self._save_dir}\n"
            f"  cleared {self._startup_removed} prior {self._prefix}#####.{self._ext} file(s)"
        )

    def _capture_filename_pattern(self) -> re.Pattern:
        return re.compile(
            "^" + re.escape(self._prefix) + r"(\d{5})\." + re.escape(self._ext) + r"$",
            re.IGNORECASE,
        )

    def _clear_existing_capture_files(self) -> int:
        """Remove prior numbered captures; keep unrelated files (e.g. calibration YAML)."""
        pat = self._capture_filename_pattern()
        removed = 0
        try:
            for name in os.listdir(self._save_dir):
                if pat.match(name):
                    try:
                        os.remove(os.path.join(self._save_dir, name))
                        removed += 1
                    except OSError:
                        pass
        except FileNotFoundError:
            pass
        return removed

    def _image_cb(self, msg: Image) -> None:
        self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _try_save_frame(self) -> tuple[bool, str]:
        if self._latest_bgr is None:
            return False, "No image received yet."

        name = f"{self._prefix}{self._next_index:05d}.{self._ext}"
        path = os.path.join(self._save_dir, name)
        ok = cv2.imwrite(path, self._latest_bgr)
        if not ok:
            return False, f"cv2.imwrite failed: {path}"

        self._next_index += 1
        return True, f"Saved {path}"

    def _auto_tick(self) -> None:
        if self._auto_remaining <= 0:
            if self._auto_timer is not None:
                self.destroy_timer(self._auto_timer)
                self._auto_timer = None
            return

        ok, msg = self._try_save_frame()
        if ok:
            self._auto_remaining -= 1
            self.get_logger().info(f"[{self._auto_target - self._auto_remaining}/{self._auto_target}] {msg}")
            if self._auto_remaining <= 0:
                if self._auto_timer is not None:
                    self.destroy_timer(self._auto_timer)
                    self._auto_timer = None
                self.get_logger().info("Capture finished; shutting down.")
                self.context.shutdown()
        else:
            self.get_logger().warn(f"Waiting for image: {msg}", throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
