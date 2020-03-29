from ..utilities.geometry import dist_between_points

"""
A heuristic function, also called simply a heuristic,
is a function that ranks alternatives in search algorithms
at each branching step based on available information to decide which branch to follow.
"""

def cost_to_go(a: tuple, b: tuple) -> float:
    """
    :param a: current location
    :param b: next location
    :return: estimated segment_cost to-go from a to b
    """
    return dist_between_points(a, b)


def path_cost(E, a: tuple, b: tuple) -> float:
    """
    Cost of the unique path from x_init to x
    :param E: edges, in form of E[child] = parent
    :param a: initial location
    :param b: goal location
    :return: segment_cost of unique path from x_init to x
    """
    cost = 0
    while not b == a:
        p = E[b]
        cost += cost_to_go(b, p)
        b = p
    return cost


def segment_cost(a: tuple, b: tuple) -> float:
    """
    :param a: tuple, start of line
    :param b: tuple, end of line
    :return: segment_cost function between a and b
    """
    return dist_between_points(a, b)