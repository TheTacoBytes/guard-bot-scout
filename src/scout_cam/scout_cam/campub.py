import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ROS 2 publishers
        self.color_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'camera/depth/image_raw', 10)

        # Timer callback to publish images
        self.timer = self.create_timer(0.1, self.publish_frames)  # 10 FPS
        self.bridge = CvBridge()

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert images to ROS messages
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
    
        # Publish messages
        self.color_publisher.publish(color_msg)
        self.depth_publisher.publish(depth_msg)

        self.get_logger().info("Published color and depth frames")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
