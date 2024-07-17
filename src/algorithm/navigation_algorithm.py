import numpy as np
import heapq
import random
# import matplotlib.pyplot as plt


class AStarGrid:
    """
    Class representing an A* grid for pathfinding.

    :param rows: (int) Number of rows in the grid.
    :param cols: (int) Number of columns in the grid.
    :param start_x: (int) X-coordinate of the start position.
    :param start_y: (int) Y-coordinate of the start position.
    :param end_x: (int) X-coordinate of the goal position.
    :param end_y: (int) Y-coordinate of the goal position.
    """

    def __init__(self, rows: int, cols: int, start_x: int, start_y: int, end_x: int, end_y: int):
        self.rows = rows
        self.cols = cols
        self.spacing = 1.0
        self.grid = np.zeros((rows, cols), dtype=int)
        self.start = (start_x, start_y)
        self.goal = (end_x, end_y)

    def generate_random_grid(self, rows: int, cols: int):
        """
        Generate a random grid with obstacles.

        :param rows: (int) Number of rows in the grid.
        :param cols: (int) Number of columns in the grid.
        """
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

    def a_star_search(self) -> list:
        """
        Perform A* search to find the shortest path from start to goal.

        :return: (list) List of tuples representing the path from start to goal.
                 Returns an empty list if no path is found.
        """
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
                new_cost = cost_so_far[current] + self.cost(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal_node, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        if goal_node not in came_from:
            print("No path found!")
            return []

        path = []
        current = goal_node
        while current != start_node:
            path.append(current)
            current = came_from[current]
        path.append(start_node)
        path.reverse()
        return path

    def update(self, grid: np.ndarray, start: tuple, goal: tuple):
        """
        Update the grid, start, and goal positions.

        :param grid: (np.ndarray) The grid to be updated.
        :param start: (tuple) The new start position as (x, y).
        :param goal: (tuple) The new goal position as (x, y).
        """
        self.grid = grid
        self.start = start
        self.goal = goal

    def get_neighbors(self, node: tuple) -> list:
        """
        Get the neighboring nodes for a given node.

        :param node: (tuple) The current node as (x, y).
        :return: (list) List of neighboring nodes as tuples.
        """
        neighbors = []
        row, col = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                neighbors.append((new_row, new_col))

        return neighbors

    def cost(self, current, next_node):
        """
        Calculate the cost of moving from the current node to the next node.

        :param current: (tuple) The current node as (x, y).
        :param next_node: (tuple) The next node as (x, y).
        :return: (int) The cost of the move.
        """
        return 1  # constant cost for moving from one cell to another

    def heuristic(self, goal: tuple, next_node: tuple) -> int:
        """
        Heuristic function for A* search. Calculates the Manhattan distance.

        :param goal: (tuple) The goal node as (x, y).
        :param next_node: (tuple) The next node as (x, y).
        :return: (int) The heuristic value.
        """
        return abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])


def create_grid_visualization(grid, spacing=1.0):
    """
    Create a visualization of the grid with points and colors.

    :param grid: (AStarGrid) The A* grid object.
    :param spacing: (float) The spacing between grid points.
    :return: (tuple) Tuple containing points, colors, edge_points, and edge_colors.
    """
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


def move(path):
    """
    Determine the move direction and power based on the given path.

    :param path: (list) List of nodes representing the path.
    :return: (tuple) Move power as a tuple.
    """
    direction_map = {
        (-1, 0): (-1.0, -1.0),  # Down
        (1, 0): (1.0, 1.0),  # Up
        (0, -1): (-0.5, 0.5),  # Left
        (0, 1): (0.5, -0.5),  # Right
        (-1, -1): (1.0, 0.5),  # Up-Right
        (-1, 1): (0.5, 1.0),  # Up-Left
        (1, -1): (-0.5, -1.0),  # Down-Left
        (1, 1): (-1.0, -0.5)  # Down-Right
    }
    current_node = path[0]
    next_node = path[1]
    move_direction = (current_node[0] - next_node[0], current_node[1] - next_node[1])
    print(f"move direction: {move_direction}, bcs: {current_node} :: {next_node}")
    move_power = direction_map.get(move_direction, None)

    return move_power

# grid_size = 1000
# grid = AStarGrid(grid_size, grid_size, start_x=0, start_y=0, end_x=grid_size - 2, end_y=grid_size - 2)
# grid.generate_random_grid(grid_size, grid_size)
#
# start = time.time()
# path = grid.a_star_search()
# end = time.time()
#
# time_taken = end - start
# print("Time taken: ", time_taken)
#
# directions = get_directions(path)
# print(path)
# print("Directions: ", directions)
#
# grid_points, grid_colors, edge_points, edge_colors = create_grid_visualization(grid, 0.1)
# path_points = [[j * 0.1, i * 0.1, 0] for i, j in path]
# path_colors = [[0, 1, 0] for _ in range(len(path_points))]
#
# visualizer = o3d.visualization.Visualizer()
# visualizer.create_window()
#
# grid_point_cloud = o3d.geometry.PointCloud()
# grid_point_cloud.points = o3d.utility.Vector3dVector(grid_points)
# grid_point_cloud.colors = o3d.utility.Vector3dVector(grid_colors)
# visualizer.add_geometry(grid_point_cloud)
#
# path_point_cloud = o3d.geometry.PointCloud()
# path_point_cloud.points = o3d.utility.Vector3dVector(path_points)
# path_point_cloud.colors = o3d.utility.Vector3dVector(path_colors)
# visualizer.add_geometry(path_point_cloud)
#
# edge_lines = [[i, i + 1] for i in range(0, len(edge_points), 2)]
# edge_line_set = o3d.geometry.LineSet()
# edge_line_set.points = o3d.utility.Vector3dVector(edge_points)
# edge_line_set.lines = o3d.utility.Vector2iVector(edge_lines)
# edge_line_set.colors = o3d.utility.Vector3dVector(edge_colors)
# visualizer.add_geometry(edge_line_set)
#
# visualizer.run()
# visualizer.destroy_window()

