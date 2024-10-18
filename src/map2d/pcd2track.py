import numpy as np
import open3d as o3d


class TrackMaker:
    """
    Class to convert point cloud data to track data.
    Converts a 2D batch of points into track data based on gradient magnitude,
    setting thresholds for drivable areas.
    NaN - indicates no data.
    0 - indicates non-drivable areas.
    1 - indicates drivable areas.

    Usage:
    track_maker = TrackMaker(threshold=0.2, has_visualization=True)
    track = track_maker.point_cloud_to_track(matrix)
    """

    def __init__(self, threshold: float = 0.1, has_visualization: bool = False):
        self.threshold = threshold
        self.has_visualization = has_visualization
        self.visualization = (
            o3d.visualization.Visualizer() if self.has_visualization else None
        )
        if self.has_visualization:
            self.visualization.create_window()

    def __del__(self):
        if self.has_visualization and self.visualization:
            self.visualization.destroy_window()

    def calculate_gradient_magnitude(self, matrix):
        """
        Calculate the gradient magnitude of the matrix.
        Ensure the matrix is at least 2x2.
        """
        if matrix.shape[0] < 2 or matrix.shape[1] < 2:
            raise ValueError("Matrix must be at least 2x2 to calculate gradients.")

        self.gradient_x, self.gradient_y = np.gradient(matrix)
        self.gradient_magnitude = np.hypot(self.gradient_x, self.gradient_y)

    def pad_matrix_if_needed(self, matrix):
        if matrix.shape[0] < 2:
            matrix = np.vstack([matrix, matrix])
        if matrix.shape[1] < 2:
            matrix = np.hstack([matrix, matrix[:, :1]])
        return matrix

    def gradient_threshold(self) -> np.array:
        """
        Apply a threshold to the gradient magnitude to determine drivable areas.
        """
        return (self.gradient_magnitude < self.threshold).astype(int)

    def preserve_nan_values(self, matrix: np.array, track: np.array) -> np.array:
        """
        Preserve NaN values in the track data based on the original matrix.
        """
        track_with_nans = track.astype(float)
        track_with_nans[np.isnan(matrix)] = np.nan
        return track_with_nans

    def point_cloud_to_track(self, matrix):
        """
        Convert point cloud matrix to track data.
        """
        matrix = self.pad_matrix_if_needed(matrix)
        self.calculate_gradient_magnitude(matrix)
        track = self.gradient_threshold()
        track_with_nans = self.preserve_nan_values(matrix, track)
        if self.has_visualization:
            self.visualize_track(track_with_nans)
        return track_with_nans

    def visualize_track(self, track: np.array):
        """
        Visualize the track using Open3D.
        """
        if not self.has_visualization:
            return
        # Convert track data to a colored point cloud
        rows, cols = track.shape
        points = []
        colors = []
        for i in range(rows):
            for j in range(cols):
                z = track[i, j] if not np.isnan(track[i, j]) else 0
                color = (
                    [0, 0, 1]
                    if np.isnan(track[i, j])
                    else ([0, 1, 0] if track[i, j] == 1 else [1, 0, 0])
                )
                points.append([j, -i, z])
                colors.append(color)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)
        self.visualization.clear_geometries()
        self.visualization.add_geometry(point_cloud)
        self.visualization.poll_events()
        self.visualization.update_renderer()


# Example usage:
# Assuming `matrix` is a 2D numpy array representing the point cloud data
# matrix = np.random.random((100, 100))
# track_maker = TrackMaker(threshold=0.2, has_visualization=True)
# track = track_maker.point_cloud_to_track(matrix)
