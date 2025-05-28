import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLOProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor')

        # YOLO model
        self.model = YOLO("yolov8n.pt")
        self.conf_threshold = 0.5  # Confidence threshold

        # ROS 2 subscribers
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(cv_image)

        # Draw results
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # Convert to numpy
            class_ids = result.boxes.cls.cpu().numpy()  # Get class IDs
            confidences = result.boxes.conf.cpu().numpy()  # Get confidence scores
            for (x1, y1, x2, y2, class_id, conf) in zip(boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3], class_ids, confidences):
                if int(class_id) == 0 and conf >= self.conf_threshold:  # Only detect 'person' class with threshold
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        cv2.imshow("YOLO Person Detections", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
