import numpy as np
import json
from collections import defaultdict


def load_config_sectors(config_path: str = "utils/sectors_conf.json") -> dict:
    with open(config_path, "r") as f:
        config = json.load(f)
    return config


def load_config_aruco(config_path: str = "utils/aruco_conf.json") -> dict:
    with open(config_path, "r") as f:
        config = json.load(f)
    return config


class DestinationsCorrectionByARUCO:
    def __init__(self):
        self.sector_size = load_config_sectors()["sector_size"]
        self.aruco_dict = load_config_aruco()

    def determineMarker(self, marker_id: int) -> str:
        """
        Determine if a marker is a destination or an entrance to a lava tube based on its ID.
        Uses predefined marker IDs to categorize the marker as either a destination
        or an entrance to a lava tube.

        :param marker_id: (int) The ID of the detected marker.
        :return: (str) 'destination' if the marker is a destination, 'entrance' if the marker is an entrance to a lava tube,
        'unknown' if the marker ID is not recognized.
        """
        key = list(filter(lambda x: self.aruco_dict[x] == int(marker_id), self.aruco_dict))

        if key[0] == "M1":
            return "Airlock_entrance"
        if key[0] == "M2":
            return "Lava_tube_entrance"
        if key[0] == "M3":
            return "Lava_tube_exit"
        return "unknown"

    def calculateSector(self, x: float, z: float) -> tuple:
        """
        Determine the sector for the given (x, z) coordinates.

        Divides the space into a grid of sectors of a specified size and determines
        which sector the given coordinates fall into.

        :param x: (float) The x coordinate.
        :param z: (float) The z coordinate.
        :return: (tuple) A tuple (sector_x, sector_z) indicating the sector numbers in the x and z directions.
        """
        sector_x = int(x // self.sector_size)
        sector_z = int(z // self.sector_size)

        return sector_x, sector_z

    def correctCoordinates(self, marker_position: list, pose: np.ndarray) -> np.ndarray:
        """
        Correct the coordinates (x, z) based on the detected marker position and the pose.

        Applies the given pose transformation to the detected ARUCO marker position
        to obtain corrected coordinates in the world frame.

        :param marker_position: (list) The [x, y, z] position of the detected marker.
        :param pose: (np.ndarray) 4x4 transformation matrix representing the pose.
        :return: (np.ndarray) Corrected [x, y, z] coordinates.
        """
        marker_position = np.array([marker_position[0], marker_position[1], marker_position[2], 1])
        correct_position = pose @ marker_position
        correct_position = correct_position[:3]
        return correct_position

    def newDestinations(self, detected_markers: dict, pose: np.ndarray) -> dict:
        """
        Compute new destinations corrected by ARUCO markers.

        Given the detected marker positions and the robot's pose, this method computes
        the corrected destination positions and determines which sectors these corrected
        positions fall into.

        :param detected_markers: (dict) Dictionary with marker id and position, e.g., {"id": 1, "position": [3.13, 2.12, 4.14]}.
        :param pose: (np.ndarray) 4x4 transformation matrix representing the robot's pose.

        :return: (dict) Dictionary containing the corrected positions and their corresponding sector numbers.
        """
        marker_positions = defaultdict(list)
        for marker in detected_markers:
            marker_positions[marker["id"]].append(marker["position"])
        corrected_positions = {}

        for marker in detected_markers:
            corrected_marker_position = self.correctCoordinates(marker['position'], pose)
            corrected_marker_position = [corrected_marker_position[0], corrected_marker_position[2],
                                         1]
            sector_x, sector_z = self.calculateSector(corrected_marker_position[0], corrected_marker_position[1])
            corrected_positions["destination"] = {
                'position': corrected_marker_position,
                'sectors': (sector_x, sector_z)
            }

        return corrected_positions


# if __name__ == "__main__":
#     correct_destination = DestinationsCorrectionByARUCO()
#     example_pose = np.array([[1, 0, 0, 2500],
#                              [0, 1, 0, 0],
#                              [0, 0, 1, 2500],
#                              [0, 0, 0, 1]])
#     detected_markers = [{"id": 67, "position": [3.13, 2.12, 4.14]}, {"id": 67, "position": [103.13, 1552.12, 8.14]}]
#
#     new_destinations = correct_destination.newDestinations(detected_markers, example_pose)
#     print(new_destinations["destination"]["sectors"])
