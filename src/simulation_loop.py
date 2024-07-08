import numpy as np
import vision.oak.reading_data as rd
import map2d.translator as m2d
from vision.oak.transform_data import assignment_to_sectors
from map2d.pcd2track import TrackMaker
import matplotlib.pyplot as plt

def generate_obstacle_map(size, num_obstacles, max_obstacle_size):
    grid = np.empty((size, size))  # Initialize grid with open space (1)

    for _ in range(num_obstacles):
        # Randomly place a rectangular obstacle
        obstacle_width = np.random.randint(10, max_obstacle_size)
        obstacle_height = np.random.randint(10, max_obstacle_size)
        x_start = np.random.randint(0, size - obstacle_width)
        y_start = np.random.randint(0, size - obstacle_height)

        grid[x_start:x_start + obstacle_width, y_start:y_start + obstacle_height] = 1

    return grid

def visualize_grid(grid):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray', interpolation='none')
    plt.title("Obstacle Map")
    plt.colorbar()
    plt.show()

if __name__ == "__main__":

    base_path = "./data1500/void_1500-47/stairs0"
    visualize = True
    S3D = rd.Simulation3D(base_path=base_path, visualize=visualize)

    res = 1000
    map_2d = np.full((res, res), np.nan)
    value_range = (-1, 1)
    # visualization_type - 0 means using open3d, any other number means using regular matplotlib

    mapper = m2d.PointCloudMapper(
        res, value_range, visualize=True, visualization_type=1
    )
    track_maker = TrackMaker(threshold=0.2, has_visualization=True)

    # simulation loop
    for image_path, depth_path, pose_path in zip(
        S3D.images, S3D.sparse_depths, S3D.poses
    ):
        rgb, pcd, point = S3D.simulation_reading_one_time(
            image_path, depth_path, pose_path
        )
        cropped_results, min_non_nan_index = assignment_to_sectors(pcd)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###
        # points = np.asarray(pcd.points)
        # track = track_maker.point_cloud_to_track(points)
        obstacle_map = generate_obstacle_map(500, 20, 100)
        #visualize_grid(obstacle_map)
        rpx = np.random.randint(10, 200)
        rpz = np.random.randint(10, 200)
        ppp = (rpz, rpx)
        map_2d = mapper.cropped_map_to_2d_map(obstacle_map, ppp)
        
