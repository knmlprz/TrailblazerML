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

    def __init__(self, has_visualization: bool = False):
        self.cords_position_json_path = "communication/api/api_data/position.json"
        self.latitude = None
        self.longitude = None
        self.prev_latitude = None
        self.prev_longitude = None

        self.x_sector = 2500
        self.y_sector = 2500

        # first time read initial position
        self.read_json_position()

        self.has_visualization = has_visualization
        self.visualization = o3d.visualization.Visualizer() if self.has_visualization else None
        if self.has_visualization:
            self.visualization.create_window()

    def __del__(self):
        if self.has_visualization and self.visualization:
            self.visualization.destroy_window()

    def update_sector(self) -> tuple[int, int]:
        self.read_json_position()

        dist_x, dist_y = self.convert_cords_to_distance_diff()
        sector_diff_x, sector_diff_y = self.convert_distance_to_sector_diff(dist_x, dist_y)
        self.x_sector += sector_diff_x
        self.y_sector += sector_diff_y

        return self.x_sector, self.y_sector

    def read_json_position(self):
        with open(self.cords_position_json_path, "r") as cords_json:
            data = json.load(cords_json)
            self.prev_latitude = self.latitude
            self.prev_longitude = self.longitude
            self.latitude = data['latitude']
            self.longitude = data['longitude']

    def convert_cords_to_distance_diff(self) -> tuple[int, int]:
        """
        Function to calculate X and Y distances (km) between two GPS coordinates
        """
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert degrees to radians
        lat1 = self.prev_latitude
        lat2 = self.latitude
        lon1 = self.prev_longitude
        lon2 = self.longitude

        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Differences in coordinates
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Mean latitude
        mean_lat = (lat1 + lat2) / 2.0

        # X distance (longitude difference adjusted by latitude)
        x = dlon * math.cos(mean_lat) * R

        # Y distance (latitude difference)
        y = dlat * R

        return x, y

    def convert_distance_to_sector_diff(self, dist_x, dist_y) -> tuple[int, int]:
        # sector is 10 cm
        KM2SECTOR_SCALE = 0.000_1
        return int(dist_x * KM2SECTOR_SCALE), int(dist_y * KM2SECTOR_SCALE)
