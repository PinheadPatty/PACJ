#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
from threading import Thread
from flask import Flask, Response

app = Flask(__name__)
output_frame = None  # Shared frame between ROS subscriber and Flask
bridge = CvBridge()

class DecompressedViewer(Node):
    def __init__(self):
        super().__init__('decompressed_viewer')
        # Subscribe to your decompressed topic
        self.subscription = self.create_subscription(
            Image,
            '/decompressed_color',
            self.callback,
            qos_profile=qos_profile_sensor_data
        )

    def callback(self, msg):
        global output_frame
        try:
            # Convert ROS Image to OpenCV image
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            output_frame = cv2.resize(cv_img, (640, 480))  # Optional resize
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def generate():
    global output_frame
    while True:
        if output_frame is None:
            continue
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', output_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    # Serve on all interfaces so your Mac can access the VM
    app.run(host='localhost', port=5000, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    node = DecompressedViewer()
    # Start Flask server in background thread
    Thread(target=flask_thread, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
