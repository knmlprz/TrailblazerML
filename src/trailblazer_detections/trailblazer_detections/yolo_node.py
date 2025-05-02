import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloDetectionNode(Node):
    """ROS2 Node for object detection using YOLO and ArUco markers."""

    def __init__(self):
        super().__init__("yolo_detection_node")
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, 10
        )
        self.get_logger().info("Subscribed to /oak/rgb/image_raw")

        # Publisher for YOLO detection result image
        self.detection_image_pub = self.create_publisher(Image, "/detections/image", 10)
        self.get_logger().info("Publishing YOLO detections to /detections/image")

        # Publisher for ArUco detection result image
        self.aruco_image_pub = self.create_publisher(
            Image, "/detections/aruco_image", 10
        )
        self.get_logger().info("Publishing ArUco detections to /detections/aruco_image")

        # Load YOLO model from ROS package share directory
        model_path = os.path.join(
            get_package_share_directory("trailblazer_detections"),
            "models",
            "urc2024.pt",
        )

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO model loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # Prepare ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Copies of image for separate detection visualizations
            yolo_image = cv_image.copy()
            aruco_image = cv_image.copy()

            # === YOLO ===
            results = self.model(yolo_image)[0]
            if results.boxes is not None and len(results.boxes) > 0:
                yolo_detections = []
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    x1, y1, x2, y2 = map(int, xyxy)
                    label = (
                        f"{self.model.names[cls]} {conf:.2f}"
                        if hasattr(self.model, "names")
                        else f"{cls} {conf:.2f}"
                    )

                    yolo_detections.append(label)

                    cv2.rectangle(yolo_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        yolo_image,
                        label,
                        (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                # Log detected YOLO objects
                self.get_logger().info(
                    f"YOLO: Detected {len(yolo_detections)} object(s): {', '.join(yolo_detections)}"
                )
            else:
                self.get_logger().info("YOLO: No objects detected.")

            # === ArUco ===
            gray = cv2.cvtColor(aruco_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                self.get_logger().info(
                    f"Detected {len(ids)} ArUco markers: {ids.flatten().tolist()}"
                )
                for i, corner in enumerate(corners):
                    corner_points = corner[0]
                    x_min = int(np.min(corner_points[:, 0]))
                    y_min = int(np.min(corner_points[:, 1]))
                    x_max = int(np.max(corner_points[:, 0]))
                    y_max = int(np.max(corner_points[:, 1]))

                    cv2.rectangle(
                        aruco_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2
                    )
                    cv2.putText(
                        aruco_image,
                        f"Aruco {ids[i][0]}",
                        (x_min, max(y_min - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )
            else:
                self.get_logger().info("No ArUco markers detected.")

            # Publish result images
            yolo_msg = self.bridge.cv2_to_imgmsg(yolo_image, encoding="bgr8")
            yolo_msg.header = msg.header
            self.detection_image_pub.publish(yolo_msg)

            aruco_msg = self.bridge.cv2_to_imgmsg(aruco_image, encoding="bgr8")
            aruco_msg.header = msg.header
            self.aruco_image_pub.publish(aruco_msg)

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
