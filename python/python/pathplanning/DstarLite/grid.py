import numpy as np
from utils import *


class GlobalOccupancyGridMap:
    def __init__(self, x_dim, y_dim, start_x: int, start_y: int, goal_x: int, goal_y: int):
        """
        set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        :param x_dim: dimension in the x direction
        :param y_dim: dimension in the y direction
        """
        # the map extents in units [m]
        self.map_extents = (x_dim, y_dim)

        # the obstacle map
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)

        # set start, current and goal
        self.start = (start_x, start_y)
        self.current = (start_x, start_y)
        self.goal = (goal_x, goal_y)

    def get_map(self):
        """
        :return: return the current occupancy grid map
        """
        return self.occupancy_grid_map

    def set_obstacle(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        print(" x, row: {} and y, col: {}".format(row, col))
        self.occupancy_grid_map[row, col] = 255

    def remove_obstacle(self, pos: (int, int)):
        """
        :param pos:
        :return:
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        print(" x, row: {} and y, col: {}".format(row, col))
        self.occupancy_grid_map[row, col] = 0

    def is_occupied(self, pos) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        return self.occupancy_grid_map[row, col] == OBSTACLE


class LocalOccupancyGridMap:
    def __init__(self, x_dim, y_dim, exploration_setting='8N'):
        """
        set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        :param exploration_setting: explore with 4-connectivity movements or 8-connectivity movements
        :param x_dim: dimension in the x direction
        :param y_dim: dimension in the y direction
        """
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.obstacles = set()
        self.exploration_setting = exploration_setting

    def in_bounds(self, cell: (int, int)) -> bool:
        """
        Checks if the provided coordinates are within
        the bounds of the local grid
        :param cell: cell position (x,y)
        :return: True if within bounds, False else
        """
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def cost(self, from_node: (int, int), to_node: (int, int)) -> float:
        """
        computes the cost of moving from one node to another
        :param from_node: (x,y)
        :param to_node: (x,y)
        :return: the computed cost of moving
        """
        if from_node is self.obstacles or to_node is self.obstacles:
            return float('inf')
        return 1.0

    def neighbors(self, cell: (int, int)) -> list:
        """
        :param cell:
        :return:
        """
        (x, y) = cell

        if self.exploration_setting == '4N':
            movements = get_movements_4n()
        else:
            movements = get_movements_8n()

        movements = filter(self.in_bounds, movements)
        return list(movements)

    def observe(self, position: (int, int), view_range: int = 2):
        """
        :param position:
        :param view_range:
        :return:
        """
        (px, py) = position
        nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
                 for y in range(py - view_range, py + view_range + 1)
                 if self.in_bounds((x, y))]
        return {node: OBSTACLE if node in self.obstacles else UNOCCUPIED for node in nodes}


class AgentViewGrid(LocalOccupancyGridMap):

    def new_obstacle(self, observation):
        """
        :param observation:
        :return:
        """
        obstacles_in_view = {node for node, nodetype in observation.items()
                             if nodetype == OBSTACLE}
        return obstacles_in_view - self.obstacles

    def update_obstacle(self, new_obstacle):
        """
        :param new_obstacle:
        :return:
        """
        self.obstacles.update(new_obstacle)
