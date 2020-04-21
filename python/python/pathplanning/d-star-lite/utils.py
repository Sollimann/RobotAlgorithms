import numpy as np
from typing import List

'''
This class used to store data in priority queue.
Comparing methods are overloaded.
'''


class Element:
    def __init__(self, key, value1, value2):
        self.key = key
        self.value1 = value1
        self.value2 = value2

    def __eq__(self, other):
        return np.sum(np.abs(self.key - other.key)) == 0

    def __ne__(self, other):
        return self.key != other.key

    def __lt__(self, other):
        return (self.value1, self.value2) < (other.value1, other.value2)

    def __le__(self, other):
        return (self.value1, self.value2) <= (other.value1, other.value2)

    def __gt__(self, other):
        return (self.value1, self.value2) > (other.value1, other.value2)

    def __ge__(self, other):
        return (self.value1, self.value2) >= (other.value1, other.value2)


def heuristic(u, v) -> float:
    """
    Helper function to compute distance between two points.
    :param u: (x,y)
    :param v: (x,y)
    :return: manhattan distance
    """
    return np.linalg.norm(u - v)


def get_movements_4n(x: int, y: int) -> List:
    """
    get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return np.array([(x + 1, y + 0),
                     (x + 0, y + 1),
                     (x - 1, y + 0),
                     (x + 0, y - 1)])


def get_movements_8n(x: int, y: int) -> List:
    """
    get all possible 8-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return np.array([(x + 1, y + 0),
                     (x + 0, y + 1),
                     (x - 1, y + 0),
                     (x + 0, y - 1),
                     (x + 1, y + 1),
                     (x - 1, y + 1),
                     (x - 1, y - 1),
                     (x + 1, y - 1)])
