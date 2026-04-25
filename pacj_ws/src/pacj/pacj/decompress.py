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

        # Drone Subscriptions & Publishers
        self.drone_color_subscription = self.create_subscription(
            CompressedImage,
            '/drone/camera/color/image_raw/compressed',
            self.drone_color_listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.drone_depth_subscription = self.create_subscription(
            CompressedImage,
            '/drone/camera/depth/image_raw/compressedDepth',
            self.drone_depth_listener_callback, 
            qos_profile=qos_profile_sensor_data)        
        
        self.drone_color_publisher = self.create_publisher(Image, '/drone/decompressed_color', qos_profile=qos_profile_sensor_data)
        self.drone_depth_publisher = self.create_publisher(Image, '/drone/decompressed_depth', qos_profile=qos_profile_sensor_data)

        # Rover Subscriptions & Publishers
        self.rover_color_subscription = self.create_subscription(
            CompressedImage,
            '/rover/color/image_raw/compressed', 
            self.rover_color_listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.rover_depth_subscription = self.create_subscription(
            CompressedImage,
            '/rover/depth/image_raw/compressedDepth', 
            self.rover_depth_listener_callback, 
            qos_profile=qos_profile_sensor_data)        
        
        self.rover_color_publisher = self.create_publisher(Image, '/rover/decompressed_color', qos_profile=qos_profile_sensor_data)
        self.rover_depth_publisher = self.create_publisher(Image, '/rover/decompressed_depth', qos_profile=qos_profile_sensor_data)

    def drone_color_listener_callback(self, msg):
        self.decompress_color(msg, self.drone_color_publisher)

    def rover_color_listener_callback(self, msg):
        self.decompress_color(msg, self.rover_color_publisher)

    def drone_depth_listener_callback(self, msg):
        self.decompress_depth(msg, self.drone_depth_publisher)

    def rover_depth_listener_callback(self, msg):
        self.decompress_depth(msg, self.rover_depth_publisher)

    def decompress_color(self, msg, publisher):
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
        publisher.publish(raw_image_msg)

    def decompress_depth(self, msg, publisher):
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
        publisher.publish(raw_image_msg)

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