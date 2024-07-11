import numpy as np
import json


def load_config(config_path: str = "/home/filip/PycharmProjects/TrailblazerML/src/utils/sectors_conf.json") -> dict:
    with open(config_path, "r") as f:
        config = json.load(f)
    return config


class DestinationsCorrectionByARUCO:
    def __init__(self):
        self.sector_size = load_config()["sector_size"]

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
        print(self.sector_size)
        return {}


if __name__ == "__main__":
    correct_destination = DestinationsCorrectionByARUCO()
    example_pose = np.array([[1, 0, 0, 2500],
                             [0, 1, 0, 0],
                             [0, 0, 1, 2500],
                             [0, 0, 0, 1]])
    detected_marker = {"id": 1, "position": [3.13, 2.12, 4.14]}

    new_destinations = correct_destination.newDestinations(detected_marker, example_pose)
    print(new_destinations)
