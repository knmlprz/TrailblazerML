import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
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

        # Find the correct path to the YOLO model
        ament_prefix_path = os.getenv("AMENT_PREFIX_PATH", "")
        if ament_prefix_path:
            install_space = ament_prefix_path.split(":")[0]
            model_path = os.path.join(
                install_space, "share", "trailblazer_detections", "models", "urc2024.pt"
            )
        else:
            # Fallback if running locally
            model_path = os.path.join(os.path.dirname(__file__), "models", "urc2024.pt")

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

    def image_callback(self, msg):
        """Process incoming image messages and perform object detection."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Perform detection
            results = self.model(cv_image)[0]

            # Log detection results
            if results.boxes is not None and len(results.boxes) > 0:
                self.get_logger().info(f"Detected {len(results.boxes)} objects.")
                for i, box in enumerate(results.boxes):
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())
                    self.get_logger().info(
                        f"Object {i}: class={cls}, conf={conf:.2f}, box={xyxy.tolist()}"
                    )
            else:
                self.get_logger().info("No objects detected.")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
