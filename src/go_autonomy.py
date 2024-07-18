import multiprocessing
from communication.api.main_api import run_api
from map2d.pcd2track import TrackMaker
from map2d.translator import PointCloudMapper
from vision.oak.camera_oak import CameraOAK
from algorithm.navigation_algorithm import AStarGrid, move
from communication.stm_com import STMCom
from vision.oak.config_oak import load_config
from vision.oak.transform_data import assignment_to_sectors, get_sector_index
from vision.qrcode.qr_code import ReadARUCOCode
from vision.qrcode.qr_code_map_correction import DestinationsCorrectionByARUCO


class GoAutonomy:
    def __init__(self, stm_com, stelite_com):
        self.config = load_config("utils/config_oak.json")
        self.camera_oak = CameraOAK(self.config, visualize=False)
        self.aruco = ReadARUCOCode()
        self.correct_aruco = DestinationsCorrectionByARUCO()
        self.point_cloud_mapper = PointCloudMapper(res=5000)
        self.stm_com = stm_com
        self.stelite_com = stelite_com
        self.track_maker = TrackMaker()
        self.end_goal = (self.point_cloud_mapper.res - 3, self.point_cloud_mapper.res - 2)
        self.a_star_grid = AStarGrid(self.point_cloud_mapper.res, self.point_cloud_mapper.res, start_x=0, start_y=0, end_x=self.end_goal[0], end_y=self.end_goal[1])
        self.start_loop = True
        self.rover_in_target = False

    def run(self):
        rgb, pcd, pose = self.camera_oak.get_data()
        isMarker, markerDict = self.aruco.read(rgb, False)
        if isMarker:
            end_goal = self.correct_aruco.newDestinations(markerDict, pose)
        print(f"pcd {pcd}")
        print(f"pose shape: {pose.shape}, pose: {pose}")
        matrix, first_sector = assignment_to_sectors(pcd)
        rover_sector = get_sector_index((pose[0, 3], pose[2, 3]))
        print(f"rover_sector {rover_sector}")
        map_01 = self.track_maker.point_cloud_to_track(matrix)
        print(f"map_01.shape {map_01.shape}")
        global_map = self.point_cloud_mapper.cropped_map_to_2d_map(map_01, first_sector)
        print(f"global_map.shape {global_map.shape}")
        self.a_star_grid.update(global_map, rover_sector, self.end_goal["destination"]["sectors"])
        path_to_destination = self.a_star_grid.a_star_search()
        moves = move(path_to_destination)
        print("move: ", moves, "\n")
        start_autonomy = self.stm_com.update(moves[0], moves[1])
        if self.a_star_grid.goal == rover_sector:
            self.rover_in_target = True

