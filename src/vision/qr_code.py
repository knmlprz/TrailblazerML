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
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters()

    def findArucoInDict(self, frame: np.ndarray):
        """
        Placeholder for a method to find ARUCO markers in the dictionary.
        """
        for arucoName, arucoDict in ARUCO_DICT.items():
            marker_dict = aruco.getPredefinedDictionary(arucoDict)
            marker_corners, marker_ids, _ = aruco.detectMarkers(
                frame, marker_dict, parameters=self.param_markers
            )
            if len(marker_corners) > 0:
                print(f"detected markers for {arucoName}")

    def read(self, frame: np.ndarray, show_visualization: bool) -> (bool, dict):
        """
        Detects ARUCO markers in the provided frame.

        :param frame: (np.ndarray) The image frame in which to detect ARUCO markers.
        :param show_visualization: (bool) Whether to display the frame.
        :return: A tuple containing a boolean indicating if markers were found,
                 and a dictionary with marker IDs and their positions if found.
        """
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            frame, self.marker_dict, parameters=self.param_markers
        )
        if show_visualization:
            self.visualize(frame, None, None)
        if marker_ids is not None:
            position = marker_corners
            if show_visualization:
                self.visualize(frame, marker_corners, marker_ids)
                return True, {"id": marker_ids, "position": position}
            return True, {"id": marker_ids, "position": position}
        return False, None

    @staticmethod
    def visualize(frame: np.ndarray, corners: tuple = None, ids: np.ndarray = None):
        """
        Visualizes provided frame.

        :param frame: (np.ndarray) The image frame on which to draw detected markers.
        :param corners: (tuple, optional) The corners of detected markers.
        :param ids: (np.ndarray, optional) The IDs of detected markers.
        """
        if corners is not None:
            frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("frame", frame_markers)
            cv2.waitKey(1)
        else:
            cv2.imshow("frame", frame)
            cv2.waitKey(1)

    @staticmethod
    def calculate_distance():
        pass
