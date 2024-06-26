import numpy as np
import open3d as o3d

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

    def __init__(self, threshold: float = 1, has_visualization: bool = False):
        self.original_map = None
        self.has_visualization = has_visualization
        self.visualization = None
        self.threshold = threshold
        self.gradient_x = None
        self.gradient_y = None
        self.gradient_magnitude = None

        if self.has_visualization:
            self.visualization = o3d.visualization.Visualizer()
            self.visualization.create_window()
            line_set = o3d.geometry.LineSet()
            self.visualization.add_geometry(line_set)
    
    def __del__(self):
        if self.visualization:
            self.visualization.destroy_window()

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
    
    def point_cloud_to_track(self, pcd: np.array):
        """
        Convert point cloud to track data.
        Args:
            None
        Returns:
            np.array: Track data.
        """
        self.original_map = pcd
        self.calculate_gradient_magnitude()
        track = self.gradient_threshold()
        track_with_NaNs = self.preserve_nan_values(track)

        if self.has_visualization:
            self.visualize_track(track_with_NaNs)

        return track_with_NaNs
    
    def visualize_track(self, track: np.array):
        """
        Visualize the track data.
        Args:
            None
        Returns:
            None
        """
        if not self.has_visualization:
            return
        
        rows, cols = track.shape

        points = []
        colors = []

        for i in range(rows):
            for j in range(cols):
                if np.isnan(track[i, j]):
                    color = [0, 0, 1]  # Blue for NaNs
                elif track[i, j]:
                    color = [0, 1, 0]  # Green for drivable areas
                else:
                    color = [1, 0, 0]  # Red for non-drivable areas

                points.append([j, -i, track[i, j] if not np.isnan(track[i, j]) else 0])
                colors.append(color)

        points = np.array(points)
        colors = np.array(colors)

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        self.visualization.clear_geometries()
        self.visualization.add_geometry(point_cloud)
        self.visualization.poll_events()
