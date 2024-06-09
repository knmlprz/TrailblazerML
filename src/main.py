import map2d.translator as m2d
from vision.oak.camera_oak import CameraOAK
import json

if __name__ == "__main__":

    # Example of how to use the classes and functions
    with open('vision/oak/config_oak.json', 'r') as json_file:
        config = json.load(json_file)
    oak = CameraOAK(config)
    rgb, pcd, pose = oak.get_data()g
    # end of example

    # Wczytaj przykładową chmurę punktów
    pcd = m2d.generate_random_point_cloud(num_points=1000, value_range=(-1, 1))
    
    # Pozycja z akcelerometru (przykładowe wartości)
    accel_position = (2, 3, 1)

    # Rzutowanie chmury punktów na mapę 2D z wizualizacją
    print(m2d.point_cloud_to_2d_map(pcd, accel_position=accel_position, visualization=True))