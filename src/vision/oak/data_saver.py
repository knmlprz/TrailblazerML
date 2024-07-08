import os
import cv2
import numpy as np
import open3d as o3d


class DataSaver:
    def __init__(self, base_path):
        """Initialize DataSaver with a base directory for storing data."""
        self.base_path = base_path

    def save_data(self, session_name, subfolder, timestamp, pose, rgb_image, depth_image, pcd):
        """Save pose, RGB and depth images, and point cloud data."""
        # Create directory paths
        session_path = os.path.join(self.base_path, session_name)
        subfolder_path = os.path.join(session_path, subfolder)
        paths = {
            'pose': os.path.join(subfolder_path, 'absolute_pose'),
            'rgb': os.path.join(subfolder_path, 'image'),
            'depth': os.path.join(subfolder_path, 'depth'),
            'pcd': os.path.join(subfolder_path, 'point_cloud')
        }

        # Ensure all directories exist
        for path in paths.values():
            os.makedirs(path, exist_ok=True)

        # Save pose data
        pose_file = os.path.join(paths['pose'], f"{timestamp}.txt")
        np.savetxt(pose_file, pose, fmt='%0.6f')

        # Save RGB image
        rgb_file = os.path.join(paths['rgb'], f"{timestamp}.png")
        cv2.imwrite(rgb_file, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))

        # Save depth image
        depth_file = os.path.join(paths['depth'], f"{timestamp}.png")
        cv2.imwrite(depth_file, depth_image)

        # Save point cloud data
        pcd_file = os.path.join(paths['pcd'], f"{timestamp}.pcd")
        o3d.io.write_point_cloud(pcd_file, pcd)

        print(f"Data saved for session {session_name}, subfolder {subfolder}, timestamp {timestamp}")


# Usage example
base_directory = "./data/"
data_saver = DataSaver(base_directory)
timestamp = "1552024072.1300"
session_name = "void_1500-47"
subfolder = "stairs0"
pose = np.eye(4)  # Example pose data
rgb_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)  # Simulated RGB image
depth_image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)  # Simulated depth image
pcd = o3d.geometry.PointCloud()  # Simulated empty point cloud for example

data_saver.save_data(session_name, subfolder, timestamp, pose, rgb_image, depth_image, pcd)
