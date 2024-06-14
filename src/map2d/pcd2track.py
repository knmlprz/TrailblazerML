import numpy as np

class TrackMaker:
    """
    todo:
        - detect empty pcd sector?
        - how to connect pcd's?

    Class to convert point cloud data to track data.
    From 2d batch of points we are calculating 2 matrices:
    - horizontal gradient
    - vertical gradient
    Then setting magnitude of gradient as a square root of sum of squares of both gradients.
    Based on that magnitude we are setting threshold to get track data,
    where łazik can drive on.
    0 - can't drive
    1 - can drive
    """
    def __init__(self):
        self.visualize = False # todo: implement visualization
        self.gradient_magnitude = None

        if self.visualize:
            self.vis = "todo: implement visualization"
    
    def __del__(self):
        if self.visualize:
            self.vis.destroy_window()

    def calculate_gradient_magnitude(self, pcd: np.array):
        """
        Calculate gradient magnitude of the point cloud.
        Args:
            pcd (np.array): Point cloud data.
        Returns:
            None
        """
        Gx, Gy = np.gradient(pcd)
        self.gradient_magnitude = np.sqrt(Gx ** 2 + Gy ** 2)
    
    def gradient_threshold(self, threshold: float = 1):
        """
        Thresholding the gradient.
        Args:
            threshold (float): Threshold value.
        Returns:
            np.array: Thresholded gradient.
        """
        print(self.gradient_magnitude < threshold)
        return self.gradient_magnitude < threshold
    
    def point_cloud_to_track(self, pcd: np.array):
        """
        Convert point cloud to track data.
        Args:
            None
        Returns:
            np.array: Track data.
        """
        self.calculate_gradient_magnitude(pcd)
        return self.gradient_threshold()
