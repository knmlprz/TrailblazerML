import os
import cv2
import numpy as np
import open3d as o3d


class Simulation3D:
    def __init__(self, base_path, visualize=False):
        """
        Class to simulate reading data from the camera in loop frame by frame .
        Args:
        base_path (str): The base path where the data is stored.
        visualize (bool): Whether to visualize the point cloud.
        Returns:
        """
        self.images, self.sparse_depths, self.poses = load_data(base_path)
        self.intrinsics_path = base_path + "/K.txt"
        self.visualize = visualize
        if visualize:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.points = []
            self.line_set = o3d.geometry.LineSet()
            self.vis.add_geometry(self.line_set)
            self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            self.pose = read_pose(self.poses[0])
            self.axis.transform(self.pose)
            self.vis.add_geometry(self.axis)

    def __del__(self):
        if self.visualize:
            self.vis.run()
            self.vis.destroy_window()

    def simulation_reading_one_time(self, rgb, depth, pose):
        """
        Function to simulate reading data from the camera.
        Args:
        rgb (np.array): The RGB image.
        depth (np.array): The depth map.
        pose (np.array): The pose matrix.
        Returns:
        rgb (np.array): The RGB image.
        pcd (open3d.geometry.PointCloud): The point cloud.
        point (np.array): The point cortinate of the camera.
        """
        rgb = read_image(rgb)
        depth = read_depth(depth)
        pose = read_pose(pose)

        color_raw = o3d.geometry.Image(rgb)
        depth_raw = o3d.geometry.Image(depth)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0,
                                                                        depth_trunc=3.0, convert_rgb_to_intensity=False)

        K = np.loadtxt(self.intrinsics_path)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb.shape[1], rgb.shape[0], K[0, 0], K[1, 1], K[0, 2], K[1, 2])
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        pcd.transform(pose)  # Apply the 4x4 transformation matrix

        self.vis.add_geometry(pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        point = pose[:3, 3]
        self.points.append(point)
        if self.visualize:
            self.points.append(point)
            self.line_set.points = o3d.utility.Vector3dVector(self.points)
            if len(self.points) > 1:
                lines = [[j, j + 1] for j in range(len(self.points) - 1)]
                self.line_set.lines = o3d.utility.Vector2iVector(lines)
                colors = [[1, 0, 0] for _ in range(len(lines))]  # czerwony kolor dla linii trajektorii
                self.line_set.colors = o3d.utility.Vector3dVector(colors)
            # transformed_axis = axis.transform(pose)
            # vis.remove_geometry(axis)
            # axis = transformed_axis
            # vis.add_geometry(axis)
            self.vis.update_geometry(self.line_set)
            self.vis.add_geometry(pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
        piont = pose[:3, 3]
        return rgb, pcd, piont


def load_data(base_path):
    """
    Load images, sparse depths and poses from the given base path.
    Args:
    base_path (str): The base path where the data is stored.
    Returns:
    images (list of str): List of paths to the images.
    sparse_depths (list of str): List of paths to the sparse depth maps.
    poses (list of str): List of paths to the pose matrices.
    """

    data_paths = {
        "images": os.path.join(base_path, "image"),
        "sparse_depths": os.path.join(base_path, "sparse_depth"),
        "poses": os.path.join(base_path, "absolute_pose")
    }

    data = {}

    for key, path in data_paths.items():

        if not os.path.exists(path):
            print(f"Directory not found: {path}")
            data[key] = []
            continue

        if key == "poses":
            files = [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.txt')]
        else:
            files = [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.png')]

        files = sorted(files)
        data[key] = files

    return data["images"], data["sparse_depths"], data["poses"]


def read_image(image_path):
    return cv2.imread(image_path, cv2.IMREAD_UNCHANGED)


def read_depth(depth_path):
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    depth = depth.astype(np.float32)
    return depth / 1000.0  # Convert depth to meters ? (assuming depth is in mm)


def read_pose(pose_path):
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


def reading_data_form_QAK():
    # funcion form qak ro read rgb depth and pose
    return  # image, sparse_depth, pose


def simulation_reading_one_time(rgb, depth, pose, vis, intrinsics_path, visualize=False, points=None, line_set=None,
                                axis=None):
    """
    Function to simulate reading data from the camera.
    Args:
    rgb (np.array): The RGB image.
    depth (np.array): The depth map.
    pose (np.array): The pose matrix.
    vis (open3d.visualization.Visualizer): The visualizer object.
    intrinsics_path (str): The path to the intrinsics file.
    visualize (bool): Whether to visualize the point cloud.
    points (list): List of points to store the trajectory.
    line_set (open3d.geometry.LineSet): The line set object.
    axis (open3d.geometry.TriangleMesh): The axis object.
    Returns:
    rgb (np.array): The RGB image.
    pcd (open3d.geometry.PointCloud): The point cloud.
    pose (np.array): The pose matrix.
    """

    color_raw = o3d.geometry.Image(rgb)
    depth_raw = o3d.geometry.Image(depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0,
                                                                    depth_trunc=3.0, convert_rgb_to_intensity=False)

    K = np.loadtxt(intrinsics_path)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb.shape[1], rgb.shape[0], K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    pcd.transform(pose)  # Apply the 4x4 transformation matrix

    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    point = pose[:3, 3]
    points.append(point)
    if visualize:
        points.append(point)
        line_set.points = o3d.utility.Vector3dVector(points)
        if len(points) > 1:
            lines = [[j, j + 1] for j in range(len(points) - 1)]
            line_set.lines = o3d.utility.Vector2iVector(lines)
            colors = [[1, 0, 0] for _ in range(len(lines))]  # czerwony kolor dla linii trajektorii
            line_set.colors = o3d.utility.Vector3dVector(colors)

        # TODO: add axis to the visualization
        # transformed_axis = axis.transform(pose)
        # vis.remove_geometry(axis)
        # axis = transformed_axis
        # vis.add_geometry(axis)
        vis.update_geometry(line_set)
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

    # vis.clear_geometries()  # This line is commented out to keep previous point clouds
    return rgb, pcd, pose[:3, 3]


def init_simulation3D(base_path, visualize=False, poses=None):
    images, sparse_depths, poses = load_data(base_path)
    intrinsics_path = base_path + "/K.txt"
    if visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        points = []
        line_set = o3d.geometry.LineSet()
        vis.add_geometry(line_set)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        pose = read_pose(poses[0])
        axis.transform(pose)
        vis.add_geometry(axis)

    return images, sparse_depths, poses, intrinsics_path, vis, points, line_set, axis


def simulation_loop_reading_data(base_path, visualize=False):
    """
    Function to simulate reading data from the camera in loop frame by frame .
    Args:
    base_path (str): The base path where the data is stored.
    visualize (bool): Whether to visualize the point cloud.
    Returns:
    none
    """
    images, sparse_depths, poses, intrinsics_path, vis, points, line_set, axis = init_simulation3D(base_path,
                                                                                                   visualize=visualize)

    for image_path, depth_path, pose_path in zip(images, sparse_depths, poses):
        rgb = read_image(image_path)
        depth = read_depth(depth_path)
        pose = read_pose(pose_path)
        rgb, pcd, point = simulation_reading_one_time(rgb, depth, pose, vis, intrinsics_path, visualize, points,
                                                      line_set, axis)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###

    if visualize:
        vis.run()
        vis.destroy_window()
