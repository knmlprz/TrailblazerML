import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from trailblazer_interfaces.msg import Detection, DetectionResult
from geometry_msgs.msg import Pose, Point
from scipy.spatial.transform import Rotation as R


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/oak/rgb/camera_info", self.camera_info_callback, 10
        )
        self.detection_image_pub = self.create_publisher(Image, "/detections/image", 10)
        self.detections_pub = self.create_publisher(Detection, "/detections", 10)

        self.get_logger().info("Subscribed to /oak/rgb/image_raw")
        self.get_logger().info("Publishing to /detections and /detections/image")

        model_path = os.path.join(
            get_package_share_directory("trailblazer_detections"),
            "models",
            "urc2024.pt",
        )
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO model loaded and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.2
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.camera_info_received = True
            self.get_logger().info("Camera info received.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detection_image = cv_image.copy()

            detection_msg = Detection()
            detection_msg.header = msg.header
            detection_msg.header.frame_id = "oak_rgb_optical_frame"
            detection_msg.image = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
            detection_msg.image.header = msg.header
            detection_msg.image.header.frame_id = "oak_rgb_optical_frame"

            # YOLO detections
            results = self.model(detection_image)[0]
            if results.boxes is not None:
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    x1, y1, x2, y2 = map(int, xyxy)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    label = self.model.names[cls] if hasattr(self.model, "names") else str(cls)

                    result = DetectionResult()
                    result.type = "yolo"
                    result.label = label
                    result.confidence = conf
                    result.center = Point(x=center_x, y=center_y, z=0.0)
                    result.width = float(x2 - x1)
                    result.height = float(y2 - y1)

                    if self.camera_matrix is not None and result.height > 0:
                        focal_length_px = self.camera_matrix[1, 1]
                        result.distance = (0.25 * focal_length_px) / result.height

                    detection_msg.detections.append(result)

                    cv2.rectangle(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(detection_image, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    if result.distance:
                        cv2.putText(detection_image, f"{result.distance:.2f} m", (x1, y2 + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # ArUco detection
            gray = cv2.cvtColor(detection_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                )
                for i, corner in enumerate(corners):
                    id = int(ids[i][0])
                    corner_points = corner[0]
                    center = np.mean(corner_points, axis=0)

                    result = DetectionResult()
                    result.type = "aruco"
                    result.label = str(id)
                    result.confidence = 1.0
                    result.center = Point(x=float(center[0]), y=float(center[1]), z=0.0)
                    result.width = float(np.linalg.norm(corner_points[0] - corner_points[1]))
                    result.height = float(np.linalg.norm(corner_points[1] - corner_points[2]))
                    result.distance = float(np.linalg.norm(tvecs[i][0]))

                    pose = Pose()
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    quat = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]
                    result.pose = pose

                    detection_msg.detections.append(result)

                    x_min = int(np.min(corner_points[:, 0]))
                    y_min = int(np.min(corner_points[:, 1]))

                    cv2.rectangle(detection_image,
                                  (x_min, y_min),
                                  (x_min + 20, y_min + 20),
                                  (255, 0, 0), 2)
                    cv2.putText(detection_image, f"Aruco {id}",
                                (x_min, y_min - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 0, 0), 2)
                    cv2.putText(detection_image, f"{result.distance:.2f} m",
                                (x_min, y_min - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 0, 255), 2)

                    cv2.drawFrameAxes(detection_image, self.camera_matrix, self.dist_coeffs,
                                      rvecs[i], tvecs[i], 0.03)

            self.detections_pub.publish(detection_msg)

            detection_img_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
            detection_img_msg.header = msg.header
            detection_img_msg.header.frame_id = "oak_rgb_optical_frame"
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
