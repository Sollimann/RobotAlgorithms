from .rrt_base import RRTBase


class RRT(RRTBase):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01):
        """
        Template of the RRT planner body (or base)
        :param X: Search space
        :param Q: list of lengths of edges add to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max of samples to take
        :param r: resolution of points to sample along the edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc)

    def rrt_search(self):
        """

        :return:
        """