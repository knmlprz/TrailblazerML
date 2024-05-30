import cv2
import numpy as np
from cv2 import aruco


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

    def findArucoInDict(self):
        """
        Placeholder for a method to find ARUCO markers in the dictionary.
        """
        pass

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
        print("corners: ", type(corners), "ids: ", type(ids))
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
