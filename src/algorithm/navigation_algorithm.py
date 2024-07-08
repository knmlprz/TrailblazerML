import open3d as o3d
import numpy as np
import time
import heapq
import random
import matplotlib.pyplot as plt


class AStarGrid:
    def __init__(self, rows, cols, start_x, start_y, end_x, end_y):
        self.rows = rows
        self.cols = cols
        self.spacing = 1.0
        self.grid = np.zeros((rows, cols), dtype=int)
        self.start = (start_x, start_y)
        self.goal = (end_x, end_y)

    def generate_random_grid(self, rows, cols):
        for i in range(rows):
            for j in range(cols):
                if (i, j) != self.start and (i, j) != self.goal:
                    k = random.randint(0, 6)
                    if k > 1:
                        self.grid[i, j] = 1
                    else:
                        self.grid[i, j] = 0
                else:
                    self.grid[i, j] = 1

    def a_star_search(self):
        start_node = self.start
        goal_node = self.goal

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
                if next_node in visited or self.grid[next_node] == 0:
                    continue
                else:
                    new_cost = cost_so_far[current] + self.cost(current, next_node)
                    if next_node not in cost_so_far and new_cost == cost_so_far[current] + 1:
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
    def update(self, grid, start):
        self.grid = grid
        self.start = start

    def get_neighbors(self, node):
        neighbors = []
        row, col = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                neighbors.append((new_row, new_col))

        return neighbors

    def cost(self, current, next_node):
        return self.grid[next_node]

    def heuristic(self, goal, next_node):
        return abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])


def create_grid_visualization(grid, spacing=1.0):
    points = []
    colors = []

    for i in range(grid.rows):
        for j in range(grid.cols):
            points.append([j * spacing, i * spacing, 0])
            if grid.grid[i, j] == 0:
                colors.append([0, 0, 0])
            elif grid.grid[i, j] == 1:
                colors.append([1, 1, 0])

    edge_colors = [[0.5, 0.5, 0.5] for _ in range((grid.rows - 1) * grid.cols + (grid.cols - 1) * grid.rows)]
    edge_points = []

    for i in range(grid.rows):
        for j in range(grid.cols - 1):
            edge_points.append([j * spacing, i * spacing, 0])
            edge_points.append([(j + 1) * spacing, i * spacing, 0])

    for j in range(grid.cols):
        for i in range(grid.rows - 1):
            edge_points.append([j * spacing, i * spacing, 0])
            edge_points.append([j * spacing, (i + 1) * spacing, 0])

    return points, colors, edge_points, edge_colors


def get_directions(path):
    directions = []
    direction_map = {
        (-1, 0): 'Up',
        (1, 0): 'Down',
        (0, -1): 'Left',
        (0, 1): 'Right',
        (-1, -1): 'Up-Left',
        (-1, 1): 'Up-Right',
        (1, -1): 'Down-Left',
        (1, 1): 'Down-Right'
    }

    for i in range(1, len(path)):
        current = path[i]
        previous = path[i - 1]
        move = (current[0] - previous[0], current[1] - previous[1])
        direction = direction_map.get(move, 'Unknown')
        directions.append(direction)

    return directions


def move(path):
    direction_map = {
        (-1, 0): (1.0, 1.0),  # Up
        (1, 0): (-1.0, -1.0),  # Down
        (0, -1): (-0.5, 0.5),  # Left
        (0, 1): (0.5, -0.5),  # Right
        (-1, -1): (0.5, 1.0),  # Up-Left
        (-1, 1): (1.0, 0.5),  # Up-Right
        (1, -1): (-0.5, -1.0),  # Down-Left
        (1, 1): (-1.0, -0.5)  # Down-Right
    }

    index_of_current_path = path.get(path[0], -1)

    if index_of_current_path == 0:
        return None

    index_of_next_path = path[index_of_current_path + 1]
    current_grid_move = (index_of_current_path[0] - index_of_next_path[0], index_of_current_path[1] - index_of_next_path[1])
    move_power = direction_map.get(current_grid_move)

    return move_power

grid_size = 1000
grid = AStarGrid(grid_size, grid_size, start_x=0, start_y=0, end_x=grid_size - 2, end_y=grid_size - 2)
grid.generate_random_grid(grid_size, grid_size)

start = time.time()
path = grid.a_star_search()
end = time.time()

time_taken = end - start
print("Time taken: ", time_taken)

directions = get_directions(path)
print(path)
print("Directions: ", directions)

grid_points, grid_colors, edge_points, edge_colors = create_grid_visualization(grid, 0.1)
path_points = [[j * 0.1, i * 0.1, 0] for i, j in path]
path_colors = [[0, 1, 0] for _ in range(len(path_points))]

visualizer = o3d.visualization.Visualizer()
visualizer.create_window()

grid_point_cloud = o3d.geometry.PointCloud()
grid_point_cloud.points = o3d.utility.Vector3dVector(grid_points)
grid_point_cloud.colors = o3d.utility.Vector3dVector(grid_colors)
visualizer.add_geometry(grid_point_cloud)

path_point_cloud = o3d.geometry.PointCloud()
path_point_cloud.points = o3d.utility.Vector3dVector(path_points)
path_point_cloud.colors = o3d.utility.Vector3dVector(path_colors)
visualizer.add_geometry(path_point_cloud)

edge_lines = [[i, i + 1] for i in range(0, len(edge_points), 2)]
edge_line_set = o3d.geometry.LineSet()
edge_line_set.points = o3d.utility.Vector3dVector(edge_points)
edge_line_set.lines = o3d.utility.Vector2iVector(edge_lines)
edge_line_set.colors = o3d.utility.Vector3dVector(edge_colors)
visualizer.add_geometry(edge_line_set)

visualizer.run()
visualizer.destroy_window()

