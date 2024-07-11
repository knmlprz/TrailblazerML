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

    def add_padding(overlay_map, padding):
        """
        Add padding to overlay map.
        Args:
            overlay_map (np.array): The overlay map.
            padding (int): Target padding.
        Returns:
            np.array: The resulting 2D map.
        """
        new_map = np.copy(overlay_map)

        index = np.argwhere(overlay_map == 1)

        for i, j in index:
            new_map[max(0, i-padding):min(new_map.shape[0], i+padding+1), 
                    max(0, j-padding):min(new_map.shape[1], j+padding+1)] = 1
        
        return new_map

    def cropped_map_to_2d_map(self, points, position=(0, 0), padding=8):
        """
        Converts a point map to global 2D map.
        Args:
            points (np.array): The point map.
            position (tuple): Position (x, z).
            padding (int): Target padding.
        Returns:
            np.array: The resulting 2D map.
        """
        # Map shape
        p_height, p_width = points.shape

        # Cut map
        if position[0] + p_width > self.res:
            p_width = self.res - position[0]
        if position[1] + p_height > self.res:
            p_height = self.res - position[1]

        overlay_map = points[:p_height, :p_width]

        # Add padding
        if padding > 0:
            overlay_map = self.add_padding(overlay_map, padding)

        self.map_2d[position[1]:p_height+position[1], position[0]:p_width+position[0]] = overlay_map

        # Visualization (optional)
        if self.visualize:
            if self.visualization_type == 0:
                self.visualize_2d_map((0,0,0))
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
        plt.ylabel('Z axis')
        plt.colorbar(label='Y value')
        
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
            y = self.map_2d[i, j]
            z = i / height
            x = j / width
            points.append([x, z, y])
            colors.append([y, y, y])

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
