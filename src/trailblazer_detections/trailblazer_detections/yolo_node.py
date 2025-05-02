import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloDetectionNode(Node):
    """ROS2 Node for object detection using YOLO and ArUco markers with pose estimation."""

    def __init__(self):
        super().__init__("yolo_detection_node")
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, 10
        )
        self.get_logger().info("Subscribed to /oak/rgb/image_raw")

        # Subscribe to camera info for pose estimation
        self.camera_info_received = False
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/oak/rgb/camera_info", self.camera_info_callback, 10
        )

        # Publisher for combined detections
        self.detection_image_pub = self.create_publisher(
            Image, "/detections/image", 10
        )
        self.get_logger().info("Publishing combined detections to /detections/image")

        # Load YOLO model
        model_path = os.path.join(
            get_package_share_directory('trailblazer_detections'),
            'models',
            'urc2024.pt'
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

        # Pose estimation parameters (camera calibration will be filled from CameraInfo)
        self.marker_length = 0.07 # 0.07 = 7 cm
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_received:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        self.camera_info_received = True
        self.get_logger().info("Camera calibration parameters received and set.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detection_image = cv_image.copy()

            # === YOLO ===
            results = self.model(detection_image)[0]
            if results.boxes is not None and len(results.boxes) > 0:
                yolo_detections = []
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    x1, y1, x2, y2 = map(int, xyxy)
                    label = f"{self.model.names[cls]} {conf:.2f}" if hasattr(self.model, 'names') else f"{cls} {conf:.2f}"
                    yolo_detections.append(label)

                    cv2.rectangle(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(detection_image, label, (x1, max(y1 - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.get_logger().info(
                    f"YOLO: Detected {len(yolo_detections)} object(s): {', '.join(yolo_detections)}"
                )
            else:
                self.get_logger().info("YOLO: No objects detected.")

            # === ArUco ===
            gray = cv2.cvtColor(detection_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                self.get_logger().info(f"Detected {len(ids)} ArUco markers: {ids.flatten().tolist()}")
                for i, corner in enumerate(corners):
                    corner_points = corner[0]
                    x_min = int(np.min(corner_points[:, 0]))
                    y_min = int(np.min(corner_points[:, 1]))
                    x_max = int(np.max(corner_points[:, 0]))
                    y_max = int(np.max(corner_points[:, 1]))

                    cv2.rectangle(detection_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                    cv2.putText(detection_image, f"Aruco {ids[i][0]}", (x_min, max(y_min - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Pose estimation
                if self.camera_info_received and self.marker_length:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                    for i in range(len(ids)):
                        cv2.drawFrameAxes(detection_image, self.camera_matrix, self.dist_coeffs,
                                          rvecs[i], tvecs[i], 0.03)
                        self.get_logger().info(
                            f"Aruco ID {ids[i][0]} pose: rvec={rvecs[i].flatten()}, tvec={tvecs[i].flatten()}"
                        )
                else:
                    self.get_logger().warn("Pose estimation skipped: camera info not yet received or marker length not set.")
            else:
                self.get_logger().info("No ArUco markers detected.")

            # Publish final image
            detection_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
            detection_msg.header = msg.header
            self.detection_image_pub.publish(detection_msg)

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
