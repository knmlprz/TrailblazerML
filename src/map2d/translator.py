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
    
    def normalize_values(self,points):
        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        points[:, 2] = 2 * (points[:, 2] - z_min) / (z_max - z_min) - 1
        return points

    def point_cloud_to_2d_map(self, point_cloud, accel_position=(0, 0, 0)):
        # Ekstrakcja punktów z chmury punktów
        points = np.asarray(point_cloud.points)

        # Translacja punktów na podstawie pozycji z akcelerometru
        translated_points = points + np.array(accel_position)

        translated_points = self.normalize_values(translated_points)

        # Określenie zakresów osi x i y
        min_x, min_y = np.min(translated_points[:, :2], axis=0)
        max_x, max_y = np.max(translated_points[:, :2], axis=0)

        # Określenie rozmiaru mapy 2D
        x_range = np.linspace(min_x, max_x, self.res)
        y_range = np.linspace(min_y, max_y, self.res)

        # Przypisanie wartości
        for x, y, z in translated_points:
            xi = np.searchsorted(x_range, x) - 1
            yi = np.searchsorted(y_range, y) - 1

            if 0 <= xi < self.res and 0 <= yi < self.res:
                self.map_2d[yi, xi] = z

        # wizualizacja
        if self.visualize:
            if self.visualization_type == 0:
                self.visualize_2d_map(accel_position)
            else:
                self.visualize_2d_map_plot()

        return self.map_2d

    def generate_random_point_cloud(self, num_points=1000, value_range=(-1, 1)):
        points = np.random.uniform(self.value_range[0], self.value_range[1], size=(num_points, 3))
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        return point_cloud

    def visualize_2d_map_plot(self):
        plt.clf()
        
        plt.imshow(self.map_2d.T, cmap='viridis', origin='lower')
        plt.title('2D Map')
        plt.xlabel('X axis')
        plt.ylabel('Y axis')
        plt.colorbar(label='Z value')
        
        plt.pause(0.01)  

    def visualize_2d_map(self,accel_position):
        height, width = self.map_2d.shape
        points = []
        colors = []

        for i in range(height):
            for j in range(width):
                z = self.map_2d[i, j]
                if not np.isnan(z):
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

                # Aktualizacja trajektorii
                point = accel_position[:3]
                self.points.append(point)
                self.points.append(point)
                self.line_set.points = o3d.utility.Vector3dVector(self.points)
                if len(self.points) > 1:
                    lines = [[j, j + 1] for j in range(len(self.points) - 1)]
                    self.line_set.lines = o3d.utility.Vector2iVector(lines)
                    colors = [[1, 0, 0] for _ in range(len(lines))]  # czerwony kolor dla linii trajektorii
                    self.line_set.colors = o3d.utility.Vector3dVector(colors)

                self.vis.update_geometry(self.line_set)
                self.vis.poll_events()
                self.vis.update_renderer()
