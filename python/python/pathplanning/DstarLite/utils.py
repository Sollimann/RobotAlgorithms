import numpy as np
import math
import heapq
from typing import List

# Define some colors
BLACK = (0, 0, 0)  # BLACK
UNOCCUPIED = (255, 255, 255)  # WHITE
GOAL = (0, 255, 0)  # GREEN
START = (255, 0, 0)  # RED
GRAY1 = (145, 145, 102)  # GRAY1
OBSTACLE = (77, 77, 51)  # GRAY2
LOCAL_GRID = (0, 0, 80)  # BLUE

colors = {
    0: UNOCCUPIED,
    1: GOAL,
    255: OBSTACLE
}


def distance(p: (int, int), q: (int, int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: manhattan distance
    """
    return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def reconstruct_path(came_from: (int, int), start: (int, int), goal: (int, int)) -> List:
    """Reconstruct a shortest path from a dictionary of back-pointers"""
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.append(start)  # optional
    path.reverse()  # optional
    return path


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


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        """
        :return:
        """
        return len(self.elements) == 0

    def put(self, item, priority):
        """
        :param item:
        :param priority:
        :return:
        """
        heapq.heappush(self.elements, (priority, item))

    def pop(self):
        """
        :return:
        """
        item = heapq.heappop(self.elements)
        return item[1]

    def first_key(self):
        """
        :return:
        """
        return heapq.nsmallest(1, self.elements)[0][0]

    def delete(self, node):
        """
        :param node:
        :return:
        """
        self.elements = [e for e in self.elements if e[1] != node]

    def __iter__(self):
        """
        :return:
        """
        for key, node in self.elements:
            yield node
