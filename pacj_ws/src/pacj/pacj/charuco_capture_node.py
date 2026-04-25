#!/usr/bin/env python3
"""
Save camera frames to disk for offline ChArUco calibration.

Subscribes to a sensor_msgs/Image topic (default /downward_camera/image_raw).
Does not run ArUco/ChArUco detection — only writes the raw BGR frame when you
call the save service (safe for headless / SSH).

Service:
  ~/save (std_srvs/Trigger) — write current frame to save_directory as
         {file_prefix}{00000}.png (index auto-increments).
"""

from __future__ import annotations

import os
import re
from typing import Optional

# Keep OpenCV from pulling GUI stacks when only imgcodecs is needed.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class CharucoCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_capture")

        self.declare_parameter("image_topic", "/downward_camera/image_raw")
        self.declare_parameter("save_directory", "~/charuco_capture")
        self.declare_parameter("file_prefix", "frame_")
        self.declare_parameter("file_extension", "png")  # png or jpg

        image_topic = str(self.get_parameter("image_topic").value)
        self._save_dir = os.path.expanduser(str(self.get_parameter("save_directory").value))
        self._prefix = str(self.get_parameter("file_prefix").value)
        ext = str(self.get_parameter("file_extension").value).lower().lstrip(".")
        if ext not in ("png", "jpg", "jpeg"):
            raise ValueError("file_extension must be png, jpg, or jpeg")
        if ext == "jpeg":
            ext = "jpg"
        self._ext = ext

        os.makedirs(self._save_dir, exist_ok=True)
        self._next_index = self._scan_next_index()
        self._bridge = CvBridge()
        self._latest_bgr: Optional[np.ndarray] = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_service(Trigger, "save", self._srv_save)

        self.get_logger().info(
            f"ChArUco capture (raw frames only)\n"
            f"  image_topic={image_topic}\n"
            f"  save_directory={self._save_dir}\n"
            f"  next_index={self._next_index:05d}\n"
            "  ros2 service call /charuco_capture/save std_srvs/srv/Trigger {}"
        )

    def _scan_next_index(self) -> int:
        pattern = re.compile(
            "^" + re.escape(self._prefix) + r"(\d{5})\." + re.escape(self._ext) + r"$",
            re.IGNORECASE,
        )
        best = -1
        try:
            for name in os.listdir(self._save_dir):
                m = pattern.match(name)
                if m:
                    best = max(best, int(m.group(1)))
        except FileNotFoundError:
            pass
        return best + 1

    def _image_cb(self, msg: Image) -> None:
        self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _srv_save(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._latest_bgr is None:
            res.success = False
            res.message = "No image received yet."
            return res

        name = f"{self._prefix}{self._next_index:05d}.{self._ext}"
        path = os.path.join(self._save_dir, name)
        ok = cv2.imwrite(path, self._latest_bgr)
        if not ok:
            res.success = False
            res.message = f"cv2.imwrite failed: {path}"
            return res

        self._next_index += 1
        res.success = True
        res.message = f"Saved {path}"
        self.get_logger().info(res.message)
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
