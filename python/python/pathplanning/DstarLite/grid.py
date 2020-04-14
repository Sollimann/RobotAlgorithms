import numpy as np


class OccupancyGridMap:
    def __init__(self, x_dim, y_dim, start_x: int, start_y: int, goal_x: int, goal_y: int):
        """
        set initial values for the map occupancy grid
        |----------> x, column
        |           (x=2,y=0)
        |
        V (y=2, x=0)
        y, row

        :param x_dim: dimension in the
        :param y_dim:
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

    def is_occupied(self, pos):
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        return self.occupancy_grid_map[row, col] == 255
