import map2d.translator as m2d
from vision.oak.camera_oak import CameraOAK

from vision.oak.config_oak import load_config
from click import getchar

if __name__ == "__main__":

    # Example of how to use the classes and functions
    config = load_config("utils/config_oak.json")
    oak = CameraOAK(config, visualize=True)
    Startloop = True
    while Startloop:
        rgb, pcd, pose = oak.get_data()
        # # Wczytaj przykładową chmurę punktów
        # pcd = m2d.generate_random_point_cloud(num_points=1000, value_range=(-1, 1))
        #
        # # Pozycja z akcelerometru (przykładowe wartości)
        # accel_position = (2, 3, 1)
        # c
        # # Rzutowanie chmury punktów na mapę 2D z wizualizacją
        # print(m2d.point_cloud_to_2d_map(pcd, accel_position=accel_position, visualization=False))
