import vision.oak.reading_data as rd
from map2d.pcd2track import TrackMaker
from map2d.translator import PointCloudMapper
from algorithm.navigation_algorithm import AStarGrid, move
from vision.oak.transform_data import assignment_to_sectors, get_sector_index
import numpy as np
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

    point_cloud_mapper = PointCloudMapper(res=5000)
    track_maker = TrackMaker(threshold=0.2, has_visualization=True)
    a_star_grid = AStarGrid(point_cloud_mapper.res, point_cloud_mapper.res, start_x=0, start_y=0,
                            end_x=point_cloud_mapper.res - 3, end_y=point_cloud_mapper.res - 2)
    # simulation loop
    for image_path, depth_path, pose_path in zip(
        S3D.images, S3D.sparse_depths, S3D.poses
    ):
        rgb, pcd, pose = S3D.simulation_reading_one_time(
            image_path, depth_path, pose_path
        )
        print(f"pcd {pcd}")
        print(f"pose shape: {pose.shape}, pose: {pose}")
        matrix, first_sector = assignment_to_sectors(pcd)
        rover_sector = get_sector_index((pose[0, 3], pose[2, 3]))
        print(f"rover_sector {rover_sector}")
        map_01 = track_maker.point_cloud_to_track(matrix)
        print(f"map_01.shape {map_01.shape}")
        global_map = point_cloud_mapper.cropped_map_to_2d_map(map_01, first_sector)
        print(f"global_map.shape {global_map.shape}")
        a_star_grid.update(global_map, rover_sector)
        path_to_destination = a_star_grid.a_star_search()
        print("move: ", move(path_to_destination), "\n")
