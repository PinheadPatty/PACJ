from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pacj",
                executable="camera_rgb_stream",
                name="camera_rgb_stream",
                output="screen",
                parameters=[
                    {
                        "video": "/dev/video0",
                        "subdev": "/dev/v4l-subdev0",
                        "raw_out": "test_raw_auto.bin",
                        "image_topic": "/camera/bgr_processed",
                        "frame_id": "camera_optical_frame",
                        "frame_rate": 5.0,
                        # Set false on headless Pi or over SSH without X11
                        "show_window": True,
                        "window_name": "PACJ camera (take_good_photo pipeline)",
                    }
                ],
            ),
        ]
    )
