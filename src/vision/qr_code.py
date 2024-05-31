import cv2
import numpy as np
from cv2 import aruco

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class ReadARUCOCode:
    """
    A class to read and process ARUCO markers from a given frame.
    """

    def __init__(self):
        """
        Initializes the ARUCO marker dictionary and detection parameters.
        """
        # dodanie self.marker_dict = Null, findArucoInDict ma znaleźć nam typ markera a nie ustawiamy na sztywno
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters()

    def findArucoInDict(self, frame: np.ndarray):
        """
        Searches for ARUCO markers in the frame using different ARUCO dictionaries.

        Iterates through a predefined set of ARUCO dictionaries and attempts to
        detect markers in the provided frame. If markers are found, it prints the
        name of the dictionary in which the markers were detected.

        :param frame: (np.ndarray) The image frame in which to search for ARUCO markers.
        """
        for arucoName, arucoDict in ARUCO_DICT.items():
            marker_dict = aruco.getPredefinedDictionary(arucoDict)
            marker_corners, marker_ids, _ = aruco.detectMarkers(
                frame, marker_dict, parameters=self.param_markers
            )
            if len(marker_corners) > 0:
                print(f"detected markers for {arucoName}")

    # TODO sprawdz poprawność return
    def read(self, frame: np.ndarray, show_visualization: bool) -> (bool, dict):
        """
        Detects ARUCO markers in the provided frame.

        :param frame: (np.ndarray) The image frame in which to detect ARUCO markers.
        :param show_visualization: (bool) Whether to display the frame.
        :return: A tuple containing a boolean indicating if markers were found,
                 and a dictionary with marker IDs and their positions if found.
        """
        position = []
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            frame, self.marker_dict, parameters=self.param_markers
        )
        if show_visualization:
            self.visualize(frame, None, None)
        # TODO if marker_id is None
        # wcześniej zrobić return
        # TODO dataclass na return
        if marker_ids is not None:
            for corners in marker_corners:
                center = self.calculate_center(corners)
                center.append(self.calculate_distance(corners, 0.05, 600))
                position.append(center)
            if show_visualization:
                self.visualize(frame, marker_corners, marker_ids, position)
                return True, {"id": marker_ids, "position": position}
            return True, {"id": marker_ids, "position": position}
        return False, None

    @staticmethod
    def visualize(frame: np.ndarray, corners: tuple = None, ids: np.ndarray = None, positions: list = None):
        """
        Visualizes provided frame.

        :param frame: (np.ndarray) The image frame on which to draw detected markers.
        :param corners: (tuple, optional) The corners of detected markers.
        :param ids: (np.ndarray, optional) The IDs of detected markers.
        """
        # TODO dodaj typowanie
        if corners is not None:
            frame = aruco.drawDetectedMarkers(frame, corners)
            for id, position in zip(ids, positions):
                center_x, center_y, z = position
                cv2.putText(frame, f"({id} {center_x:.2f}, {center_y:.2f}, {z:.2f})", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)

        cv2.imshow("frame", frame)

    @staticmethod
    def calculate_distance(marker_corners, marker_length, focal_length):
        # TODO popraw nazewnictwo
        # dodanie typów
        pixel_width = np.linalg.norm(marker_corners[0][0] - marker_corners[0][1])
        distance = (marker_length * focal_length) / pixel_width
        return distance

    @staticmethod
    def calculate_center(marker_corners):
        center_x = np.mean(marker_corners[0][:, 0])
        center_y = np.mean(marker_corners[0][:, 1])
        return [center_x, center_y]
    