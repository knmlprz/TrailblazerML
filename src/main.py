import multiprocessing
from communication.api.main_api import run_api
from map2d.pcd2track import TrackMaker
from map2d.translator import PointCloudMapper
from vision.oak.camera_oak import CameraOAK
from algorithm.navigation_algorithm import AStarGrid, move
from communication.stm_com import STMCom
from vision.oak.config_oak import load_config
from vision.oak.transform_data import assignment_to_sectors, get_sector_index
import open3d as o3d
import numpy as np

def main_process():
    config = load_config("utils/config_oak.json")
    camera_oak = CameraOAK(config, visualize=False)
    point_cloud_mapper = PointCloudMapper(res=5000)
    stm_com = STMCom(port="/dev/ttyACM0")
    track_maker = TrackMaker()
    a_star_grid = AStarGrid(point_cloud_mapper.res, point_cloud_mapper.res, start_x=0, start_y=0, end_x=point_cloud_mapper.res - 3, end_y=point_cloud_mapper.res - 2)
    start_loop = True
    while start_loop:
        rgb, pcd, pose = camera_oak.get_data()
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
        moves = move(path_to_destination)
        print("move: ", moves, "\n")

        start_autonumy = stm_com.update(moves[0], moves[1])

if __name__ == "__main__":
    api_process = multiprocessing.Process(target=run_api)
    api_process.start()
    main_process()
    api_process.join()