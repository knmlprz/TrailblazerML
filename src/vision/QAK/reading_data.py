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
    return depth / 1000.0


def visualize_point_cloud(rgb_image, depth_image, camera_intrinsics_path):
    color_raw = o3d.geometry.Image(rgb_image)
    depth_raw = o3d.geometry.Image(depth_image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1.0,
                                                                    depth_trunc=3.0, convert_rgb_to_intensity=False)

    K = np.loadtxt(camera_intrinsics_path)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(rgb_image.shape[1], rgb_image.shape[0], K[0, 0], K[1, 1], K[0, 2],
                                                  K[1, 2])

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # Wizualizacja
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    base_path = "../../../data/exp2/birthplace_of_internet"
    images, sparse_depths, poses = load_data(base_path)

    print("Loaded images:")
    for img in images:
        print(img)
    print("\nLoaded sparse depths:")
    for depth in sparse_depths:
        print(depth)
    print("\nLoaded poses:")
    for pose in poses:
        print(pose)

    for i in range(min(5, len(images))):
        rgb = read_image(images[i])
        depth = read_depth(sparse_depths[i])
        visualize_point_cloud(rgb, depth,
                              "../../../data/exp2/birthplace_of_internet/K.txt")