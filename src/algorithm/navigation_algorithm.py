import open3d as o3d
import numpy as np
import time
from queue import PriorityQueue
import random
import heapq
import matplotlib.pyplot as plt


class AStarGrid:
    def __init__(self, rows, cols, spacing=1.0):
        self.rows = rows
        self.cols = cols
        self.spacing = spacing
        self.grid = np.zeros((rows, cols), dtype=int)
        self.start = (0, 0)
        self.goal = (rows - 3, cols - 2)

    def generate_random_grid(self, rows, cols):

        # Define obstacles and their cost
        self.obstacles = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2)]
        self.obstacle_cost = 100  # High cost for obstacles

        # Set obstacle costs
        for obstacle in self.obstacles:
            self.grid[obstacle] = self.obstacle_cost

        # Assign random costs to each cell
        for i in range(rows):
            for j in range(cols):
                if (i, j) not in self.obstacles and (i, j) != self.start and (i, j) != self.goal:
                    self.grid[i, j] = random.randint(0, 1000)

    def a_star_search(self):
        start_node = (0, 0)
        goal_node = (self.rows - 3, self.cols - 2)

        frontier = []
        heapq.heappush(frontier, (0, start_node))

        came_from = {start_node: None}
        cost_so_far = {start_node: 0}

        visited = set()

        while frontier:
            _, current = heapq.heappop(frontier)
            visited.add(current)

            if current == goal_node:
                break

            for next_node in self.get_neighbors(current):
                if next_node in visited:
                    continue

                new_cost = cost_so_far[current] + self.cost(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal_node, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        path = []
        current = goal_node
        while current != start_node:
            path.append(current)
            current = came_from[current]
        path.append(start_node)
        path.reverse()
        return path

    def get_neighbors(self, node):
        neighbors = []
        row, col = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                neighbors.append((new_row, new_col))

        return neighbors

    def cost(self, current, next_node):
        return self.grid[next_node]  # Cost is the value in the grid cell

    def heuristic(self, goal, next_node):
        return abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])  # Manhattan distance


def create_grid_visualization(grid, spacing=1.0):
    points = []
    colors = []

    # Add points and colors for grid cells
    for i in range(grid.rows):
        for j in range(grid.cols):
            points.append([j * spacing, i * spacing, 0])
            if grid.grid[i, j] == grid.obstacle_cost:
                colors.append([1, 0, 0])  # Red for obstacles
            elif (i, j) == grid.start:
                colors.append([0, 0, 1])  # Blue for start
            elif (i, j) == grid.goal:
                colors.append([0, 1, 0])  # Green for goal
            else:
                # Scale the color based on the cost value
                scaled_cost = grid.grid[i, j] / 1000.0  # Normalize to range [0, 1]
                colors.append([1 - scaled_cost, 1 - scaled_cost, 1])  # White for walkable cells

    # Add points and colors for edges (horizontal and vertical lines)
    edge_colors = [[0.5, 0.5, 0.5] for _ in range((grid.rows - 1) * grid.cols + (grid.cols - 1) * grid.rows)]
    edge_points = []

    # Horizontal edges
    for i in range(grid.rows):
        for j in range(grid.cols - 1):
            idx = i * grid.cols + j
            edge_points.append([j * spacing, i * spacing, 0])
            edge_points.append([(j + 1) * spacing, i * spacing, 0])

    # Vertical edges
    for j in range(grid.cols):
        for i in range(grid.rows - 1):
            idx = (grid.rows - 1) * grid.cols + j * (grid.rows - 1) + i
            edge_points.append([j * spacing, i * spacing, 0])
            edge_points.append([j * spacing, (i + 1) * spacing, 0])

    return points, colors, edge_points, edge_colors

# grid_array_to_test = np.array([200,400,500,1000])
# grid_time = np.array([])
# for i in grid_array_to_test:
#     grid = AStarGrid(i, i, 0.1)
#     grid.generate_random_grid(i, i)
#     start = time.time()
#     path = grid.a_star_search()
#     end = time.time()
#     time_taken = end - start
#     grid_time = np.append(grid_time, time_taken)
#     print("Time taken: ", end - start)
#
# plt.plot(grid_array_to_test, grid_time)
# plt.xlabel("grid size")
# plt.ylabel("time ")
# plt.title("Trailblazer algorithm time")
# plt.show()

# Create visualization data
# grid_points, grid_colors, edge_points, edge_colors = create_grid_visualization(grid, spacing)
# path_points = [[j * spacing, i * spacing, 0] for i, j in path]
# path_colors = [[0, 1, 0] for _ in range(len(path_points))]

# Create Open3D visualizer and add geometries
# visualizer = o3d.visualization.Visualizer()
# visualizer.create_window()

# Create grid point cloud
#grid_point_cloud = o3d.geometry.PointCloud()
#grid_point_cloud.points = o3d.utility.Vector3dVector(grid_points)
#grid_point_cloud.colors = o3d.utility.Vector3dVector(grid_colors)
#visualizer.add_geometry(grid_point_cloud)

# Create path point cloud
# path_point_cloud = o3d.geometry.PointCloud()
# path_point_cloud.points = o3d.utility.Vector3dVector(path_points)
# path_point_cloud.colors = o3d.utility.Vector3dVector(path_colors)
# visualizer.add_geometry(path_point_cloud)

# Create edge line set
#edge_line_set = o3d.geometry.LineSet()
#edge_line_set.points = o3d.utility.Vector3dVector(edge_points)
#edge_line_set.lines = o3d.utility.Vector2iVector(np.array([[i, i + 1] for i in range(0, len(edge_points), 2)]))
#edge_line_set.colors = o3d.utility.Vector3dVector(edge_colors)
#visualizer.add_geometry(edge_line_set)

# Run the visualizer
# visualizer.run()
# visualizer.destroy_window()
