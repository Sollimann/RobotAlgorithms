import numpy as np
import math
from typing import List

"""
A heuristic function, also called simply a heuristic,
is a function that ranks alternatives in search algorithms
at each branching step based on available information to
decide which branch to follow.
"""


def distance(p: (int, int), q: (int, int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: manhattan distance
    """
    return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def is_outside_grid(x: int, y: int, x_lim: int, y_lim: int) -> bool:
    """
    determines if the provided coordinates are within the bounds
    of the grid
    :param x: int, x coord
    :param y: int, y coord
    :param x_lim: int, x boundary
    :param y_lim: int, y boundary
    :return: true means it unvalid index, else false
    """
    return x < 0 or x >= x_lim or y < 0 or y >= y_lim


def get_movements_4n() -> List:
    """
    get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]


def get_movements_8n() -> List:
    """
    get all possible 8-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]
