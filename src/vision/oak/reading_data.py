import os
import cv2
import numpy as np
import open3d as o3d


class Simulation3D:
    def __init__(self, base_path: str, 5):
        """
        Class to simulate reading data from the camera in a loop, frame by frame.
        Args:
            base_path (str): The base path where the data is stored.
            visualize (bool): Whether to visualize the point cloud.
        """
        self.base_path = base_path
        self.visualize = visualize
        try:
            self.images, self.sparse_depths, self.poses = load_data(base_path)
            self.intrinsics_path = os.path.join(base_path, "K.txt")
            if visualize:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window()
                self.points = []
                self.line_set = o3d.geometry.LineSet()
                self.vis.add_geometry(self.line_set)
                self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                self.vis.add_geometry(self.axis)
        except Exception as e:
            print(f"Failed to initialize Simulation3D: {e}")

    def __del__(self):
        if self.visualize:
            self.vis.run()
            self.vis.destroy_window()

    def simulation_reading_one_time(self, path_image: str, path_depth: str, path_pose: str) -> (
            np.ndarray, o3d.geometry.PointCloud, np.ndarray):

        """
        Function to simulate reading data from the camera.
        Args:
            path_image (str): Path to the RGB image file.
            path_depth (str): Path to the sparse depth file.
            path_pose (str): Path to the pose file.
        Returns:
            tuple: A tuple containing the RGB image, point cloud, and camera position.
        """
        try:
            rgb = read_image(path_image)
            depth = read_depth(path_depth)
            pose = read_pose(path_pose)

            color_raw = o3d.geometry.Image(rgb)
            depth_raw = o3d.geometry.Image(depth)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_raw, depth_raw, depth_scale=1.0, depth_trunc=3.0, convert_rgb_to_intensity=False)

            K = np.loadtxt(self.intrinsics_path)
            intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb.shape[1], rgb.shape[0], K[0, 0], K[1, 1], K[0, 2],
                                                          K[1, 2])
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            pcd.transform(pose)

            point = pose[:3, 3]
            if self.visualize:
                self.vis.add_geometry(pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
                self.points.append(point)
                self.line_set.points = o3d.utility.Vector3dVector(self.points)
                if len(self.points) > 1:
                    lines = [[j, j + 1] for j in range(len(self.points) - 1)]
                    self.line_set.lines = o3d.utility.Vector2iVector(lines)
                    colors = [[1, 0, 0] for _ in range(len(lines))]
                    self.line_set.colors = o3d.utility.Vector3dVector(colors)
                self.axis.transform(pose)
                self.vis.update_geometry(self.axis)
                self.vis.update_geometry(self.line_set)
                self.vis.poll_events()
                self.vis.update_renderer()
            return rgb, pcd, point
        except Exception as e:
            print(f"Failed during simulation: {e}")
            return None, None, None


def load_data(base_path: str) -> (list, list, list):
    """
    Load images, sparse depths, and poses from the given base path.
    Args:
        base_path (str): The base path where the data is stored.
    Returns:
        tuple: A tuple containing lists of paths to the images, sparse depths, and poses.
    """
    data_paths = {
        "images": os.path.join(base_path, "image"),
        "sparse_depths": os.path.join(base_path, "sparse_depth"),
        "poses": os.path.join(base_path, "absolute_pose")
    }

    data = {}

    for key, path in data_paths.items():
        if not os.path.exists(path):
            raise FileNotFoundError(f"Directory not found: {path}")
        files = [os.path.join(path, f) for f in sorted(os.listdir(path))
                 if (key == 'poses' and f.endswith('.txt')) or (key != 'poses' and f.endswith('.png'))]
        data[key] = files

    return data["images"], data["sparse_depths"], data["poses"]


def read_image(image_path: str) -> np.ndarray:
    """
    Read an image from the given path.
    Args:
        image_path (str): The path to the image file.
    Returns:
        np.ndarray: The image as a numpy array.
    """
    return cv2.imread(image_path, cv2.IMREAD_UNCHANGED)


def read_depth(depth_path=str, scale: float = 1000.0) -> np.ndarray:
    """
    Read a depth image from the given path and scale it to meters.
    Args:
        depth_path (str): The path to the depth image file.
        scale (float): The scale factor to convert the depth values.
    Returns:
        np.ndarray: The depth image as a numpy array in meters.
    """
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    depth = depth.astype(np.float32)
    return depth / scale  # Convert depth to meters ? (assuming depth is in mm)


def read_pose(pose_path: str) -> np.ndarray:
    """Reads a single pose matrix from a text file.
    Args:
    pose_path (str): Path to the text file containing the pose matrix.
    Returns:
    pose (np.array): The pose matrix as a 4x4 numpy array.
    """
    pose = np.loadtxt(pose_path, dtype=np.float64)
    if pose.shape == (3, 4):  # Convert 3x4 matrix to 4x4 by adding a row for homogeneous coordinates
        pose = np.vstack((pose, np.array([0, 0, 0, 1], dtype=np.float64)))
    return pose
