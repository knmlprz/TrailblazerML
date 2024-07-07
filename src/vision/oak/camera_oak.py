import depthai as dai
import open3d as o3d
import numpy as np
import cv2
from .imu_tracker import ImuTracker
from .camera_hendler import CameraHendler
import matplotlib.pyplot as plt
from .transform_data import assignment_to_sectors


class CameraOAK:
    """Class for initializing camera and getting data from it."""

    def __init__(self, config: dict, visualize: bool = False):
        """Initialize the CameraOAK class.
        Args:
            config (dict): The configuration dictionary.
        """
        self.handler = CameraHendler(config)
        self.device = dai.Device(self.handler.pipeline)
        self.set_laser_IrFloodLight()
        self.base_time = None
        self.imu_tracker = ImuTracker()
        self.visualize = visualize
        self.init_visualizer()
        self.pose = np.eye(4)
        self.cv_color_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.pcd = o3d.geometry.PointCloud()
        self.i = 0

    def __del__(self):
        if self.visualize:
            self.vis.destroy_window()

    def set_laser_IrFloodLight(self):
        self.device.setIrLaserDotProjectorIntensity(0.9)
        self.device.setIrFloodLightIntensity(0.9)

    def init_visualizer(self) -> None:
        """Initialize the visualizer if needed."""
        if self.visualize:
            self.vis = o3d.visualization.VisualizerWithKeyCallback()
            self.vis.create_window()
            self.vis.register_key_callback(32, self.toggle_geometry_addition)  # Space bar to toggle
            self.add_geometry = True  # Flag to control geometry addition
            self.line_points = []
            self.line_set = o3d.geometry.LineSet()
            self.vis.add_geometry(self.line_set)
            self.origin_arrow = o3d.geometry.TriangleMesh.create_coordinate_frame(size=150, origin=[0, 0, 0])
            self.vis.add_geometry(self.origin_arrow)

    def get_data(self) -> (np.ndarray, o3d.geometry.PointCloud, np.ndarray):
        """Get processed data like the RGB image, point cloud, and pose.
        Returns:
            tuple:(np.ndarray, o3d.geometry.PointCloud, np.ndarray) A tuple containing the RGB image, point cloud, and camera position.
        """
        imu_queue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        pc_queue = self.device.getOutputQueue(name="out", maxSize=5, blocking=False)
        depth_queue = self.device.getOutputQueue(name="depth", maxSize=5, blocking=False)

        imu_data = imu_queue.tryGet()
        if imu_data:
            self.handle_imu_data(imu_data)

        pc_message = pc_queue.tryGet()
        if pc_message:
            self.handle_pc_data(pc_message)
            self.visualize_data()

        depth_message = depth_queue.tryGet()
        if depth_message:
            self.handle_depth_data(depth_message)

        return self.cv_color_frame, self.pcd, self.pose

    def handle_imu_data(self, imu_data):
        """Process IMU data."""
        imu_packets = imu_data.packets
        for imu_packet in imu_packets:
            accelero_values = imu_packet.acceleroMeter
            rotation_vector = imu_packet.rotationVector
            current_time = imu_packet.acceleroMeter.getTimestampDevice()

            if self.base_time is None:
                self.base_time = current_time

            delta_t = (current_time - self.base_time).total_seconds()
            self.base_time = current_time
            self.pose = self.imu_tracker.update(
                [accelero_values.x, accelero_values.y, accelero_values.z],
                [
                    rotation_vector.i,
                    rotation_vector.j,
                    rotation_vector.k,
                    rotation_vector.real,
                ],
                delta_t,
            )

    def handle_pc_data(self, pc_message):
        """Process point cloud data."""
        self.pcd = o3d.geometry.PointCloud()
        in_point_cloud = pc_message["pcl"]
        points = in_point_cloud.getPoints().astype(np.float64)
        self.pcd.points = o3d.utility.Vector3dVector(points)
        in_color = pc_message["rgb"]
        self.cv_color_frame = in_color.getCvFrame()

        if self.pose is not None:
            self.line_points.append(self.pose[:3, 3])

        if not self.pcd.is_empty():
            self.pcd = self.pcd.voxel_down_sample(voxel_size=100)

    def visualize_data(self):
        """Visualize the point cloud data with control over geometry addition."""
        if self.visualize:
            print(self.add_geometry)
            cvRGBFrame = cv2.cvtColor(self.cv_color_frame, cv2.COLOR_BGR2RGB)

            # Załóżmy, że self.pcd to już istniejąca chmura punktów z punktami w self.pcd.points
            points = np.asarray(self.pcd.points)
            y_values = points[:, 1]  # Wartości Y

            # Normalizacja Y
            y_min, y_max = np.min(y_values), np.max(y_values)
            y_normalized = (y_values - y_min) / (y_max - y_min)

            # Użycie mapy kolorów z Matplotlib
            cmap = plt.get_cmap("viridis")  # Możesz zmienić na inną mapę kolorów
            colors = cmap(y_normalized)[:, :3]  # Pobranie tylko RGB, ignorowanie kanału alfa

            # Przypisanie kolorów do chmury punktów
            self.pcd.colors = o3d.utility.Vector3dVector(colors)

            self.i += 1
            if self.i > 10 and self.add_geometry:
                self.vis.add_geometry(self.pcd)

            self.vis.poll_events()
            self.vis.update_renderer()
            self.update_trajectory()
            if self.add_geometry:
                self.pcd.clear()

    def handle_depth_data(self, depth_message):
        """Process depth data and combine it with RGB data for visualization."""
        if self.visualize:
            depth_frame = depth_message.getFrame()
            depth_frame_color = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
            depth_frame_color = cv2.applyColorMap(depth_frame_color.astype(np.uint8), cv2.COLORMAP_JET)

            # Assume self.cv_color_frame is the current RGB frame updated from the point cloud message

            # Resize depth to match RGB image dimensions
            depth_frame_color_resized = cv2.resize(depth_frame_color,
                                                   (self.cv_color_frame.shape[1], self.cv_color_frame.shape[0]))

            # Combine images horizontally
            combined_image = cv2.hconcat([self.cv_color_frame, depth_frame_color_resized])

            cv2.imshow("Combined Depth and RGB", combined_image)
            cv2.waitKey(1)

    def toggle_geometry_addition(self, vis):
        """Toggle the addition of geometry on key press."""
        self.add_geometry = not self.add_geometry
        print(f"Geometry addition toggled to {'ON' if self.add_geometry else 'OFF'}.")

    def update_trajectory(self):
        """Update the line set for the trajectory visualization."""
        self.line_set.points = o3d.utility.Vector3dVector(self.line_points)
        if len(self.line_points) > 1:
            lines = [[j, j + 1] for j in range(len(self.line_points) - 1)]
            self.line_set.lines = o3d.utility.Vector2iVector(lines)
            colors = [[1, 0, 0] for _ in range(len(lines))]
            self.line_set.colors = o3d.utility.Vector3dVector(colors)
        self.vis.update_geometry(self.line_set)
        self.vis.poll_events()
        self.vis.update_renderer()
