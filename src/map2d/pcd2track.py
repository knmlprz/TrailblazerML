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
    def __init__(self, pcd: np.array, threshold: float = 1, has_visualization: bool = False):
        self.original_map = pcd
        self.has_visualization = has_visualization # todo: implement visualization
        self.visualization = None
        self.threshold = threshold
        self.gradient_x = None
        self.gradient_y = None
        self.gradient_magnitude = None

        if self.has_visualization:
            self.visualization = "todo: implement visualization"
    
    def __del__(self):
        if True:
            # self.visualization.destroy_window()
            pass

    def calculate_gradient_magnitude(self):
        """
        Calculate gradient magnitude of the point cloud.
        Args:
            pcd (np.array): Point cloud data.
        Returns:
            None
        """
        self.gradient_x, self.gradient_y = np.gradient(self.original_map)
        self.gradient_magnitude = np.sqrt(self.gradient_x ** 2 + self.gradient_y ** 2)
    
    def gradient_threshold(self):
        """
        Thresholding the gradient.
        Args:
            threshold (float): Threshold value.
        Returns:
            np.array: Thresholded gradient.
        """
        return (self.gradient_magnitude < self.threshold).astype(int)
    
    def preserve_nan_values(self, array: np.array):
        array = array.astype(float)
        array[np.isnan(self.original_map)] = np.nan
        return array
    
    def point_cloud_to_track(self):
        """
        Convert point cloud to track data.
        Args:
            None
        Returns:
            np.array: Track data.
        """
        self.calculate_gradient_magnitude()
        track = self.gradient_threshold()
        return self.preserve_nan_values(track)
