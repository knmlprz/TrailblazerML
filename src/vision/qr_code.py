import cv2
import numpy as np
from cv2 import aruco


class ReadARUCOCode:
    def __init__(self):
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters()

    def findArucoInDict(self):
        pass

    def read(self, frame: np.ndarray, show_visualization: bool) -> (bool, dict):
        marker_corners, marker_ids, _ = aruco.detectMarkers(frame, self.marker_dict, parameters=self.param_markers)
        if show_visualization:
            self.visualize(frame, None, None)
        if marker_ids is not None:
            position = marker_corners
            if show_visualization:
                self.visualize(frame, marker_corners, marker_ids)
                return True, {'id': marker_ids, 'position': position}
            return True, {'id': marker_ids, 'position': position}
        return False, None

    @staticmethod
    def visualize(frame: np.ndarray, corners: tuple = None, ids: np.ndarray = None):
        print("corners: ", type(corners), "ids: ", type(ids))
        if corners is not None:
            frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow('frame', frame_markers)
            cv2.waitKey(1)
        else:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)

    @staticmethod
    def calculate_distance():
        pass

