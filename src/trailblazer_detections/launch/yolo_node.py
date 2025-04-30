import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
from ultralytics import YOLO


class YoloDetectionNode(Node):
    """ROS 2 Node for performing object detection using a YOLO model."""

    def __init__(self):
        """Initialize the YOLO detection node."""
        super().__init__("yolo_detection_node")
        self.bridge = CvBridge()

        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, 10
        )
        self.get_logger().info("Subscription to /oak/rgb/image_raw created.")

        # Publisher only for annotated image
        self.detection_image_pub = self.create_publisher(
            Image, "/detections/image", 10
        )
        self.get_logger().info("Publisher to /detections/image created.")

        # Path to model
        script_dir = os.path.dirname(os.path.abspath(__file__))  # .../launch/
        model_path = os.path.abspath(os.path.join(script_dir, "..", "models", "urc2024.pt"))

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

    def image_callback(self, msg):
        """Process image messages, perform YOLO detection, draw results, and publish annotated image."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Run YOLO detection
            results = self.model(cv_image)[0]

            if results.boxes is not None and len(results.boxes) > 0:
                self.get_logger().info(f"Detected {len(results.boxes)} objects.")
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    # Draw box and label
                    x1, y1, x2, y2 = map(int, xyxy)
                    label = f"{self.model.names[cls]} {conf:.2f}" if hasattr(self.model, 'names') else f"{cls} {conf:.2f}"
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, max(y1 - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                self.get_logger().info("No objects detected.")

            # Publish annotated image
            detection_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            detection_img_msg.header = msg.header
            self.detection_image_pub.publish(detection_img_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
