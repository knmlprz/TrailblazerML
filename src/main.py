from map2d.pcd2track import TrackMaker
from map2d.translator import PointCloudMapper
from vision.oak.camera_oak import CameraOAK

from vision.oak.config_oak import load_config
from vision.oak.transform_data import assignment_to_sectors
import open3d as o3d
import numpy as np

if __name__ == "__main__":

    # Example of how to use the classes and functions
    config = load_config("utils/config_oak.json")
    oak = CameraOAK(config, visualize=True)
    point_cloud_mapper = PointCloudMapper()
    trackmaker = TrackMaker()
    Startloop = True
    while Startloop:
        rgb, pcd, pose = oak.get_data()
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
        map_01 = trackmaker.point_cloud_to_track(matrix)
        global_map = point_cloud_mapper.cropped_map_to_2d_map(map_01, first_sector)



