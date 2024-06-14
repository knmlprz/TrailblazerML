import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class PointCloudMapper:
    def __init__(self, res=1000, value_range=(-1, 1), visualize=False, visualization_type=1):
        self.res = res
        self.value_range = value_range
        self.map_2d = np.full((res, res), np.nan)
        self.visualize = visualize
        self.visualization_type = visualization_type

        self.ac = (0, 0, 0)

        if self.visualize:
            if self.visualization_type == 0:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window()
                self.points = []
                self.line_set = o3d.geometry.LineSet()
                self.vis.add_geometry(self.line_set)

    def __del__(self):
        if self.visualize:
            self.vis.destroy_window()
    
    def normalize_values(self,points):
        """
        Normalizes the z-values of the given points to be within the range [-1, 1].
        Args:
            points (np.array): Points extracted from the point cloud.
        Returns:
            np.array: The array of normalized points.
        """
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        normalized_points = (points - min_vals) / (max_vals - min_vals)
        return normalized_points

    def point_cloud_to_2d_map(self, point_cloud, accel_position=(0, 0, 0)):
        """
        Converts a point cloud to a 2D map centered around (0, 0).
        Args:
            point_cloud (open3d.geometry.PointCloud): The point cloud.
            accel_position (tuple): Acceleration position.
        Returns:
            np.array: The resulting 2D map.
        """
        # Extract points from the point cloud
        points = np.asarray(point_cloud.points)

        self.ac += accel_position

        # Translate points based on the acceleration position
        translated_points = points + np.array(self.ac)

        #translated_points = self.normalize_values(translated_points)

        # Determine the range of x and y axes
        min_x, min_y = np.min(translated_points[:, :2], axis=0)
        max_x, max_y = np.max(translated_points[:, :2], axis=0)

        # Calculate center of the point cloud
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2

        # Determine the size of the 2D map
        half_size = max(max_x - center_x, max_y - center_y)
        x_range = np.linspace(center_x - half_size, center_x + half_size, self.res)
        y_range = np.linspace(center_y - half_size, center_y + half_size, self.res)

        # Assign values to the 2D map
        # self.map_2d.fill(0)  # Reset map before filling
        for x, y, z in translated_points:
            xi = np.searchsorted(x_range, x) - 1 + int(self.ac[0])
            yi = np.searchsorted(y_range, y) - 1 + int(self.ac[1])

            if 0 <= xi < self.res and 0 <= yi < self.res:
                self.map_2d[yi, xi] = z

        # Visualization (optional)
        if self.visualize:
            if self.visualization_type == 0:
                self.visualize_2d_map(accel_position)
            else:
                self.visualize_2d_map_plot()

        return self.map_2d

    def visualize_2d_map_plot(self):
        """
        Visualizes a 2D map using Matplotlib.
        Args:
            None
        Returns:
            None
        """
        plt.clf()
        
        plt.imshow(self.map_2d.T, cmap='viridis', origin='lower')
        plt.title('2D Map')
        plt.xlabel('X axis')
        plt.ylabel('Y axis')
        plt.colorbar(label='Z value')
        
        # Pause to update the plot
        plt.pause(0.01)  

    def visualize_2d_map(self, accel_position):
        """
        Visualizes a 2D map using Open3D.
        Args:
            accel_position (np.array): Acceleration position.
        Returns:
            None
        """
        height, width = self.map_2d.shape
        points = []
        colors = []

        non_nan_indices = np.where(~np.isnan(self.map_2d))
        for i, j in zip(non_nan_indices[0], non_nan_indices[1]):
            z = self.map_2d[i, j]
            x = i / height
            y = j / width
            points.append([x, y, z])
            colors.append([z, z, z])

        if len(points) > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            if self.visualize:
                self.vis.add_geometry(pcd)
                self.vis.poll_events()
                self.vis.update_renderer()

                # Update the trajectory
                point = accel_position[:3]
                self.points.append(point)
                self.points.append(point)
                self.line_set.points = o3d.utility.Vector3dVector(self.points)
                if len(self.points) > 1:
                    lines = [[j, j + 1] for j in range(len(self.points) - 1)]
                    self.line_set.lines = o3d.utility.Vector2iVector(lines)
                    colors = [[1, 0, 0] for _ in range(len(lines))]  # red color for trajectory lines
                    self.line_set.colors = o3d.utility.Vector3dVector(colors)

                self.vis.update_geometry(self.line_set)
                self.vis.poll_events()
                self.vis.update_renderer()