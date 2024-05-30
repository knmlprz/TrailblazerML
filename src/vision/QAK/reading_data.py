import os
import cv2
import numpy as np
import open3d as o3d


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


def read_imu_data(imu_path):
    """Read IMU data from file and convert it to a rotation matrix or directly use it if already provided as such.
    Argrs:
    imu_path (str): Path to the text file containing the IMU data.
    Returns:
    imu_data (np.array): The IMU data as a rotation matrix.
    """
    imu_data = np.loadtxt(imu_path, dtype=np.float64)
    # Assuming IMU data is in quaternion format [w, x, y, z]
    # Convert quaternion to rotation matrix
    return quaternion_to_rotation_matrix(imu_data)


def add_trajectory_point(line_set, point):
    """Visualize a trajectory point by adding it to the line set.
    Args:
    line_set (o3d.geometry.LineSet): The line set to which the point will be added.
    point (np.array): The 3D point to be added to the line set.
    Returns:
    None
    """
    if len(line_set.points) > 0:
        line_set.lines.append([len(line_set.points) - 1, len(line_set.points)])
    line_set.points.append(point)


def visualize_point_clouds_with_trajectory(images, sparse_depths, poses, intrinsics_path):
    """
    Visualize point clouds with camera's trajectory.
    Args:
    images (list of str): List of paths to the images.
    sparse_depths (list of str): List of paths to the sparse depth maps.
    poses (list of str): List of paths to the pose matrices.
    intrinsics_path (str): Path to the camera intrinsics file.
    Returns:
    None
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for image_path, depth_path, pose_path in zip(images, sparse_depths, poses):
        rgb = read_image(image_path)
        depth = read_depth(depth_path)
        pose = read_pose(pose_path)

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
        # vis.clear_geometries()  # This line is commented out to keep previous point clouds

    poses = [np.loadtxt(pose, dtype=np.float64) for pose in poses]
    points = [pose[:3, 3] for pose in poses]
    points = np.array(points)

    # Tworzenie linii trajektorii
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    colors = [[1, 0, 0] for i in range(len(lines))]  # czerwony kolor dla linii trajektorii
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Dodanie trajektorii do wizualizacji i uruchomienie
    vis.add_geometry(line_set)

    vis.run()
    vis.destroy_window()





def reading_data_form_QAK():
    # funcion form qak ro read rgb depth and pose
    pass
    return image, sparse_depth, pose


def symulation_reading_one_time(rgb, depth, pose, vis, intrinsics_path, visualize=False):
    color_raw = o3d.geometry.Image(rgb)
    depth_raw = o3d.geometry.Image(depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0,
                                                                    depth_trunc=3.0, convert_rgb_to_intensity=False)

    K = np.loadtxt(intrinsics_path)
    K = np.eye(4    )
    intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb.shape[1], rgb.shape[0], K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    pcd.transform(pose)  # Apply the 4x4 transformation matrix

    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()


    # vis.clear_geometries()  # This line is commented out to keep previous point clouds
    return rgb, pcd, pose[:3, 3]


def symulation_loop_reading_data(base_path, visualize=False):
    images, sparse_depths, poses = load_data(base_path)
    intrinsics_path = base_path + "/K.txt"
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    points = []
    line_set = o3d.geometry.LineSet()
    arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1,
                                                   cone_height=0.04)
    arrow.compute_vertex_normals()
    arrow.paint_uniform_color([1, 0, 0])  # Kolor czerwony


    for image_path, depth_path, pose_path in zip(images, sparse_depths, poses):
        rgb = read_image(image_path)
        depth = read_depth(depth_path)
        pose = read_pose(pose_path)
        # reading function
        rgb, pcd, point = symulation_reading_one_time(rgb, depth, pose, vis, intrinsics_path, visualize)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###
        points.append(point)
        if visualize:
            points.append(point)
            line_set.points = o3d.utility.Vector3dVector(points)
            if len(points) > 1:
                lines = [[j, j + 1] for j in range(len(points) - 1)]
                line_set.lines = o3d.utility.Vector2iVector(lines)
                colors = [[1, 0, 0] for _ in range(len(lines))]  # czerwony kolor dla linii trajektorii
                line_set.colors = o3d.utility.Vector3dVector(colors)

            arrow.translate(point)
            arrow.rotate(pose[:3, :3], center=point)
            vis.add_geometry(arrow)

            vis.add_geometry(line_set)
            vis.update_geometry(line_set)
            vis.add_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

    if visualize:
        vis.run()
        vis.destroy_window()




if __name__ == "__main__":
    base_path = "../../../data1500//void_1500-47/stairs0"
    symulation_loop_reading_data(base_path, visualize=True)
