import numpy as np
from rtree import index

from ..utilities.geometry import es_points_along_line
from ..utilities.obstacle_generation import obstacle_generator


class SearchSpace(object):
    def __init__(self, dimension_lengths, O=None):
        """
        Initialize search space
        :param dimension_lengths: range of each dimension
        :param O: list of obstacles
        """
        # sanity check
        if len(dimension_lengths) < 2:
            raise Exception("must have at least 2 dimensions")
        self.dimensions = len(dimension_lengths)  # number of dimensions

        # sanity check(s)
        if any(len(i) != 2 for i in dimension_lengths):
            raise Exception("Dimensions can only have a start and end")
        if any(i[0] >= i[1] for i in dimension_lengths):
            raise Exception("Dimension start must be less than dimension end")
        self.dimension_lengths = dimension_lengths  # length of each dimension
        p = index.Property()  # index property is a container that contains a number of settable index properties
        p.dimension = self.dimensions
        if O is None:
            self.obs = index.Index(interleaved=True, properties=p)  # initialize spatial indexing with Rtrees
        else:
            # r-tree representation of obstacles
            # sanity check
            if any(len(o) / 2 != len(dimension_lengths) for o in O):
                raise Exception("Obstacle has incorrect dimension definition")
            if any(o[i] >= o[int(i + len(o) / 2)] for o in O for i in range(int(len(o) / 2))):
                raise Exception("Obstacle start must be less than obstacle end")
            self.obs = index.Index(obstacle_generator(O), interleaved=True, properties=p)

    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        return self.obs.count(x) == 0

    def sample_free(self):
        """
        Sample a location within X_free. Commonly known as seeding
        :return: random location within X_free
        """
        while True:
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        points = es_points_along_line(start, end, r)
        coll_free = all(map(self.obstacle_free, points))
        return coll_free

    def sample(self):
        """
        Return a random location within X, called X_rand
        :return: random location within X, called X_rand (not necessarily X_free)
        """
        x = np.random.uniform(self.dimension_lengths[:, 0], self.dimension_lengths[:, 1])
        return tuple(x)