import os
import cv2
import numpy as np
import open3d as o3d


def load_data(base_path):
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
    return depth / 1000.0  # Convert depth to meters

def read_pose(pose_path):
    """Reads a single pose matrix from a text file."""
    pose = np.loadtxt(pose_path, dtype=np.float64)
    if pose.shape == (3, 4):  # Convert 3x4 matrix to 4x4 by adding a row for homogeneous coordinates
        pose = np.vstack((pose, np.array([0, 0, 0, 1], dtype=np.float64)))
    return pose

def read_imu_data(imu_path):
    """Read IMU data from file and convert it to a rotation matrix or directly use it if already provided as such."""
    imu_data = np.loadtxt(imu_path, dtype=np.float64)
    # Assuming IMU data is in quaternion format [w, x, y, z]
    # Convert quaternion to rotation matrix
    return quaternion_to_rotation_matrix(imu_data)


import open3d as o3d


def visualize_point_clouds_with_trajectory(images, depths, poses, intrinsics_path, imu_data):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    trajectory = o3d.geometry.LineSet()  # Przygotowanie do rysowania trajektorii

    for image_path, depth_path, pose_path, imu_path in zip(images, depths, poses, imu_data):
        rgb = cv2.imread(image_path, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        pose = read_pose(pose_path)
        imu_rotation = read_imu_data(imu_path)

        color_raw = o3d.geometry.Image(rgb)
        depth_raw = o3d.geometry.Image(depth)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0,
                                                                        depth_trunc=3.0, convert_rgb_to_intensity=False)

        K = np.loadtxt(intrinsics_path)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb.shape[1], rgb.shape[0], K[0, 0], K[1, 1], K[0, 2], K[1, 2])
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

        # Zastosuj rotację z IMU i translację z pozycji
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = imu_rotation
        transformation_matrix[:3, 3] = pose[:3, 3]
        pcd.transform(transformation_matrix)

        vis.add_geometry(pcd)

        # Dodaj punkt trajektorii
        add_trajectory_point(trajectory, transformation_matrix[:3, 3])

        vis.poll_events()
        vis.update_renderer()
        vis.clear_geometries()

    vis.add_geometry(trajectory)
    vis.run()
    vis.destroy_window()


def add_trajectory_point(line_set, point):
    """Dodaje nowy punkt do trajektorii."""
    if len(line_set.points) > 0:
        line_set.lines.append([len(line_set.points) - 1, len(line_set.points)])
    line_set.points.append(point)


def visualize_point_clouds_with_trajectory(images, sparse_depths, poses, intrinsics_path):
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
        vis.clear_geometries()  # Clear the previous point cloud before adding a new one

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    base_path = "../../../data/exp2/birthplace_of_internet"
    images = sorted([os.path.join(base_path, "image", f) for f in os.listdir(os.path.join(base_path, "image")) if
                     f.endswith('.png')])
    sparse_depths = sorted(
        [os.path.join(base_path, "sparse_depth", f) for f in os.listdir(os.path.join(base_path, "sparse_depth")) if
         f.endswith('.png')])
    poses = sorted(
        [os.path.join(base_path, "absolute_pose", f) for f in os.listdir(os.path.join(base_path, "absolute_pose")) if
         f.endswith('.txt')])

    visualize_point_clouds_with_trajectory(images, sparse_depths, poses,
                                           "../../../data/exp2/birthplace_of_internet/K.txt")