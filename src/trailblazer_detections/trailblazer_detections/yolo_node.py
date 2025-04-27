import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
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

        # Publisher for processed detection image
        self.detection_image_pub = self.create_publisher(
            Image, "/detections/image", 10
        )
        self.get_logger().info("Publisher to /detections/image created.")

        # Publisher for detection boxes
        self.detection_boxes_pub = self.create_publisher(
            Detection2DArray, "/detections/boxes", 10
        )
        self.get_logger().info("Publisher to /detections/boxes created.")

        # Find the correct path to the YOLO model
        ament_prefix_path = os.getenv("AMENT_PREFIX_PATH", "")
        if ament_prefix_path:
            install_space = ament_prefix_path.split(":")[0]
            model_path = os.path.join(
                install_space, "share", "trailblazer_detections", "models", "urc2024.pt"
            )
        else:
            model_path = os.path.join(os.path.dirname(__file__), "models", "urc2024.pt")

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

    def image_callback(self, msg):
        """Process incoming image messages, perform object detection, and publish detections."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Perform detection
            results = self.model(cv_image)[0]

            # Create empty Detection2DArray
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            # Draw detections on the image
            if results.boxes is not None and len(results.boxes) > 0:
                self.get_logger().info(f"Detected {len(results.boxes)} objects.")
                for i, box in enumerate(results.boxes):
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    x1, y1, x2, y2 = xyxy
                    x_center = (x1 + x2) / 2.0
                    y_center = (y1 + y2) / 2.0
                    width = x2 - x1
                    height = y2 - y1

                    # Create Detection2D
                    detection = Detection2D()
                    detection.bbox.center.position.x = float(x_center)
                    detection.bbox.center.position.y = float(y_center)
                    detection.bbox.size_x = float(width)
                    detection.bbox.size_y = float(height)


                    # Fill hypotheses
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)

                    
                    detections_msg.detections.append(detection)

                    # Draw rectangle on the image
                    x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                    label = f"{self.model.names[cls]} {conf:.2f}" if hasattr(self.model, 'names') else f"{cls} {conf:.2f}"
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, max(y1 - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            else:
                self.get_logger().info("No objects detected.")

            # Publish detection boxes
            self.detection_boxes_pub.publish(detections_msg)

            # Publish processed image
            detection_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            detection_img_msg.header = msg.header
            self.detection_image_pub.publish(detection_img_msg)

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
