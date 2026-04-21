#!/usr/bin/env python3
"""
Stream V4L2 RG10 through the same pipeline as take_good_photo.py (repo root).

TUNING (Bayer, roll, seam repair, AWB, etc.) lives in take_good_photo.py — edit that file.
This node only exposes device paths, rate, topic name, optional window, and frame_id.
"""

import importlib
import sys
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _import_take_good_photo():
    """Repo layout: <repo>/take_good_photo.py next to <repo>/pacj_ws/.../this file."""
    here = Path(__file__).resolve()
    for anc in here.parents:
        candidate = anc / "take_good_photo.py"
        if candidate.is_file():
            root = str(anc)
            if root not in sys.path:
                sys.path.insert(0, root)
            return importlib.import_module("take_good_photo")
    raise ImportError(
        f"take_good_photo.py not found in parents of {here}. "
        "Expected at workspace root (sibling to pacj_ws/)."
    )


class CameraRgbStreamNode(Node):
    def __init__(self):
        super().__init__("camera_rgb_stream")

        self._tgp = _import_take_good_photo()
        self._bridge = CvBridge()

        self.declare_parameter("video", "/dev/video0")
        self.declare_parameter("subdev", "/dev/v4l-subdev0")
        self.declare_parameter("raw_out", "test_raw_auto.bin")
        self.declare_parameter("image_topic", "/camera/bgr_processed")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("frame_rate", 5.0)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "PACJ camera (take_good_photo pipeline)")

        topic = self.get_parameter("image_topic").get_parameter_value().string_value
        qos = 10
        self._pub = self.create_publisher(Image, topic, qos)

        rate = max(0.25, float(self.get_parameter("frame_rate").value))
        period = 1.0 / rate
        self.create_timer(period, self._on_timer)

        self._show = bool(self.get_parameter("show_window").value)
        self._win = self.get_parameter("window_name").get_parameter_value().string_value

        self.get_logger().info(
            f"Streaming BGR on {topic} at ~{rate:.2f} Hz; "
            f"TUNING from take_good_photo.py (roll={self._tgp.RAW_VERTICAL_ROLL_ROWS}). "
            f"show_window={self._show}"
        )

    def _on_timer(self) -> None:
        video = self.get_parameter("video").get_parameter_value().string_value
        subdev = self.get_parameter("subdev").get_parameter_value().string_value
        raw_out = self.get_parameter("raw_out").get_parameter_value().string_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        tgp = self._tgp
        if not tgp._capture_one_frame(video, subdev, raw_out):
            self.get_logger().warning("V4L2 capture failed (missing raw file?)")
            return

        try:
            raw, _dbg, _w, _h = tgp._load_raw_for_processing(raw_out, video)
        except ValueError as e:
            self.get_logger().error(f"Raw load failed: {e}")
            return

        try:
            bgr = tgp._raw_to_bgr(raw, int(tgp.RAW_VERTICAL_ROLL_ROWS))
        except Exception as e:
            self.get_logger().error(f"Pipeline failed: {e}")
            return

        msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self._pub.publish(msg)

        if self._show:
            cv2.imshow(self._win, bgr)
            cv2.waitKey(1)

    def destroy_node(self):
        if self._show:
            try:
                cv2.destroyWindow(self._win)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRgbStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
