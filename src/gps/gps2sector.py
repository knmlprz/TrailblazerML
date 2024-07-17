import numpy as np
import open3d as o3d
import json
import math


class SectorFinder:
    """
    Class to convert gps cords to map sector

    Usage:
    sf = SectorFinder()
    while True:
        x_sector, y_sector = sf.update_sector()
    """

    def __init__(self):
        self.cords_position_json_path = "../utils/position.json"
        self.latitude = None
        self.longitude = None
        self.initial_latitude, self.initial_longitude = self.read_json_position()

        self.INITIAL_X_SECTOR = 2500
        self.INITIAL_Y_SECTOR = 2500

    def update_sector(self) -> tuple[int, int]:
        self.latitude, self.longitude = self.read_json_position()

        dist_x, dist_y = self.convert_cords_to_distance_diff()
        sector_diff_x, sector_diff_y = self.convert_distance_to_sector_diff(dist_x, dist_y)
        x_sector = self.INITIAL_X_SECTOR + sector_diff_x
        y_sector = self.INITIAL_Y_SECTOR + sector_diff_y

        return x_sector, y_sector

    def update_pose_from_gps(self, pose: np.ndarray(4) = np.eye(4)) -> np.ndarray(4):
        if pose is None:
            pose = np.eye(4)
        x_sector, y_sector = self.update_sector()
        pose[0, 3] = x_sector
        pose[2, 3] = y_sector
        return pose

    def read_json_position(self) -> tuple[int, int]:
        with open(self.cords_position_json_path, "r") as cords_json:
            data = json.load(cords_json)
            return data['latitude'], data['longitude']

    def convert_cords_to_distance_diff(self) -> tuple[int, int]:
        """
        Function to calculate X and Y distances (km) between two GPS coordinates
        """
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert degrees to radians
        lat1 = self.initial_latitude
        lat2 = self.latitude
        lon1 = self.initial_longitude
        lon2 = self.longitude

        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Differences in coordinates
        diff_lat = lat2 - lat1
        diff_lon = lon2 - lon1

        # Mean latitude
        mean_lat = (lat1 + lat2) / 2.0

        # X distance (longitude difference adjusted by latitude)
        x = diff_lon * math.cos(mean_lat) * R

        # Y distance (latitude difference)
        y = diff_lat * R

        return x, y

    def convert_distance_to_sector_diff(self, dist_x, dist_y) -> tuple[int, int]:
        # sector is 10 cm
        KM2SECTOR_SCALE = 1000_0
        return int(dist_x * KM2SECTOR_SCALE), int(dist_y * KM2SECTOR_SCALE)
