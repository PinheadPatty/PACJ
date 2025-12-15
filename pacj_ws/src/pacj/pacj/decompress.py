#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor')
        self.bridge = CvBridge()

        self.color_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed', 
            self.color_listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.depth_subscription = self.create_subscription(
            CompressedImage,
            '/camera/depth/image_raw/compressedDepth', 
            self.depth_listener_callback, 
            qos_profile=qos_profile_sensor_data)        
        
        
        self.color_publisher = self.create_publisher(Image, '/decompressed_color', qos_profile=qos_profile_sensor_data)
        self.depth_publisher = self.create_publisher(Image, '/decompressed_depth', qos_profile=qos_profile_sensor_data)

        
        
    def color_listener_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_image is None:
                self.get_logger().warn("Color decoding returned None")
                return
        except Exception as e:
            self.get_logger().error(f"Error decompressing color image: {e}")
            return

        raw_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        raw_image_msg.header = msg.header
        self.color_publisher.publish(raw_image_msg)

    def depth_listener_callback(self, msg):
        try:
            raw_data = np.frombuffer(msg.data, np.uint8)

            depth_header_size = 12
            if 'compressedDepth' in msg.format:
                raw_data = raw_data[depth_header_size:]
            
            cv_image = cv2.imdecode(raw_data, cv2.IMREAD_UNCHANGED)
            if cv_image is None:
                self.get_logger().warn("Depth decoding failed (cv_image is None). Check topic format.")
                return

        except Exception as e:
            self.get_logger().error(f"Error decompressing depth image: {e}")
            return

        raw_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        raw_image_msg.header = msg.header
        self.depth_publisher.publish(raw_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()