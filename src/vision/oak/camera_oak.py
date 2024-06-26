import depthai as dai
import open3d as o3d
import numpy as np
# from vision.oak.imu_tracker import ImuTracker
# from vision.oak.camera_hendler import CameraHendler
# from vision.oak.test import ImuTracker


class CameraOAK:
    """class for init camera and get data from it."""

    def __init__(self, config: dict, visualize: bool = False):
        """ Initialize the CameraOAK class.
        Args:
            config (dict): The configuration dictionary.
        """
        self.handler = CameraHendler(config)
        self.device = dai.Device(self.handler.pipeline)
        self.base_time = None
        self.imu_tracker = ImuTracker(visualize=True)  # TODOvisualize=self.visualize

        self.visualize = visualize
        self.init_visualizer()

    def __del__(self):
        if self.visualize:
            self.vis.destroy_window()

    def init_visualizer(self) -> None:
        """ Initialize the visualizer if needed."""
        if self.visualize:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.line_points = []
            self.line_set = o3d.geometry.LineSet()
            self.vis.add_geometry(self.line_set)

    def get_data(self):
        """Get data from the camera.

        Returns:
            tuple: (cvColorFrame, pcd, pose) containing the RGB image, point cloud, and camera pose.
        """
        imu_queue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        pc_queue = self.device.getOutputQueue(name="out", maxSize=4, blocking=False)
        pose = None
        pcd = o3d.geometry.PointCloud()
        imu_data = imu_queue.tryGet()
        pc_message = pc_queue.tryGet()

        if imu_data:
            imu_packets = imu_data.packets
            for imu_packet in imu_packets:
                accelero_values = imu_packet.acceleroMeter
                gyro_values = imu_packet.gyroscope
                rotation_vector = imu_packet.rotationVector
                current_time = imu_packet.acceleroMeter.getTimestampDevice()

                if self.base_time is None:
                    self.base_time = current_time

                delta_t = (current_time - self.base_time).total_seconds()
                self.base_time = current_time
                # print([accelero_values.x, accelero_values.y, accelero_values.z],
                #     [rotation_vector.i, rotation_vector.j, rotation_vector.k, rotation_vector.real],
                #     delta_t)

                pose = self.imu_tracker.update(
                    [accelero_values.x, accelero_values.y, accelero_values.z],
                    [gyro_values.x, gyro_values.y, gyro_values.z],
                    [rotation_vector.i, rotation_vector.j, rotation_vector.k, rotation_vector.real],
                    delta_t
                )
                # print(pose)

        if pc_message:
            in_point_cloud = pc_message["pcl"]
            points = in_point_cloud.getPoints().astype(np.float64)
            pcd.points = o3d.utility.Vector3dVector(points)
            in_color = pc_message["rgb"]
            cv_color_frame = in_color.getCvFrame()

            if self.visualize:
                if pose is not None:
                    self.line_points.append(pose[:3, 3])
                    pcd.transform(pose)
                self.vis.add_geometry(pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
                self.update_trajectory()

            return cv_color_frame, pcd, pose

        return None, None, None

    def update_trajectory(self):
        """ Update the line set for the trajectory visualization."""
        self.line_set.points = o3d.utility.Vector3dVector(self.line_points)
        if len(self.line_points) > 1:
            lines = [[j, j + 1] for j in range(len(self.line_points) - 1)]
            self.line_set.lines = o3d.utility.Vector2iVector(lines)
            colors = [[1, 0, 0] for _ in range(len(lines))]
            self.line_set.colors = o3d.utility.Vector3dVector(colors)
        self.vis.update_geometry(self.line_set)
        self.vis.poll_events()
        self.vis.update_renderer()


import numpy as np
import matplotlib.pyplot as plt


def generate_obstacle_map(size, num_obstacles, max_obstacle_size):
    grid = np.ones((size, size), dtype=int)  # Initialize grid with open space (1)

    for _ in range(num_obstacles):
        # Randomly place a rectangular obstacle
        obstacle_width = np.random.randint(10, max_obstacle_size)
        obstacle_height = np.random.randint(10, max_obstacle_size)
        x_start = np.random.randint(0, size - obstacle_width)
        y_start = np.random.randint(0, size - obstacle_height)

        grid[x_start:x_start + obstacle_width, y_start:y_start + obstacle_height] = 0

    return grid


def visualize_grid(grid):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray', interpolation='none')
    plt.title("Obstacle Map")
    plt.colorbar()
    plt.show()


# Parameters
size = 5000
num_obstacles = 200  # Number of obstacles
max_obstacle_size = 200  # Maximum size of any obstacle

# Generate and visualize the map
obstacle_map = generate_obstacle_map(size, num_obstacles, max_obstacle_size)
visualize_grid(obstacle_map)

