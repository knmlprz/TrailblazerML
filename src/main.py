from map2d.pcd2track import TrackMaker
from map2d.translator import PointCloudMapper
from vision.oak.camera_oak import CameraOAK
from algorithm.navigation_algorithm import AStarGrid

from vision.oak.config_oak import load_config
from vision.oak.transform_data import assignment_to_sectors, get_sector_index
import open3d as o3d
import numpy as np

if __name__ == "__main__":

    # Example of how to use the classes and functions

    config = load_config("utils/config_oak.json")
    camera_oak = CameraOAK(config, visualize=True)
    point_cloud_mapper = PointCloudMapper()
    track_maker = TrackMaker()
    a_star_grid = AStarGrid(point_cloud_mapper.res, point_cloud_mapper.res, start_x=0, start_y=0, end_x=point_cloud_mapper.res - 3, end_y=point_cloud_mapper.res - 2)
    start_loop = True
    while start_loop:
        rgb, pcd, pose = camera_oak.get_data()
        print(type(pcd))
        if pcd.is_empty():
            print("Chmura punktów jest pusta. Tworzenie domyślnej chmury punktów...")
            # Tworzymy przykładowe punkty, np. 10 punktów na płaszczyźnie XY
            num_points = 10
            np_points = np.random.rand(num_points, 3)  # Losowe punkty w zakresie [0, 1]
            np_points[:, 2] = 0  # Ustawienie współrzędnej Z na 0 dla wszystkich punktów

            # Utworzenie nowej chmury punktów
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np_points)


        matrix, first_sector  = assignment_to_sectors(pcd)
        rover_sector = get_sector_index((pose[0, 3], pose[2, 3]))
        map_01 = track_maker.point_cloud_to_track(matrix)
        global_map = point_cloud_mapper.cropped_map_to_2d_map(map_01, first_sector)
        a_star_grid.update(global_map, rover_sector)
        path_to_destination =a_star_grid.a_star_search()



