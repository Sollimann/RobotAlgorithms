from collections import deque
from functools import partial

from utils import *
from grid import OccupancyGridMap, AgentViewGrid


class DstarLite(object):
    def __init__(self, graph, start: (int, int), goal: (int, int), view_range=2):
        # init the graphs
        #self.local_map = AgentViewGrid(x_dim=graph.x_dim, y_dim=graph.y_dim)
        #self.global_map: OccupancyGridMap = graph
        pass
