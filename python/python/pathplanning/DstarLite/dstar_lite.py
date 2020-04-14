from collections import deque
from functools import partial

from utils import *
from grid import OccupancyGridMap, AgentViewGrid


class DstarLite(object):
    def __init__(self, world, start: (int, int), goal: (int, int), view_range=2):
        # init the graphs
        self.est_global_map = OccupancyGridMap(x_dim=world.x_dim,
                                               y_dim=world.y_dim)

        # real_graph
        self.gt_global_map: OccupancyGridMap = world

        # graph
        self.est_local_map = AgentViewGrid(x_dim=world.x_dim,
                                           y_dim=world.y_dim)

        # init variables
        self.view_range = view_range
        self.position = start
        self.goal = goal
        self.back_pointers = {self.goal: None}

        # procedure initialize
        self.U = PriorityQueue()
        self.km = 0
        self.RHS_VALS = self.G_VALS = {}  # empty set
        # self.rhs[goal] = 0
        self.U.put(self.goal, self.calculate_key(self.goal))

    def g(self, node: (int, int)):
        """
        :param node:
        :return:
        """
        return self.G_VALS.get(node, float('inf'))

    def rhs(self, node: (int, int)):
        """
        :param node:
        :return:
        """
        return self.RHS_VALS.get(node, float('inf')) if node != self.goal else 0

    def calculate_key(self, node: (int, int)):
        """
        procedure CalculateKey(s)
        :param node:
        :return:
        """
        g_rhs = min([self.g(node), self.rhs(node)])
        return g_rhs + distance(node, self.position) + self.km, g_rhs

    def update_vertex(self, node: (int, int)):
        """
        procedure UpdateVertex(u)
        :param node:
        :return:
        """
        if node != self.goal:
            self.RHS_VALS[node] = self.calculate_rhs(node)
        self.U.delete(node)
        if self.g(node) != self.rhs(node):
            self.U.put(node, self.calculate_key(node))

    def update_vertices(self, nodes: List[(int, int)]):
        """
        :param nodes:
        :return:
        """
        for node in nodes:
            self.update_vertex(node=node)

    def lookahead_cost(self, node, neighbor):
        """
        :param node:
        :param neighbor:
        :return:
        """
        return self.g(neighbor) + self.est_local_map.cost(neighbor, node)

    def lowest_cost_neighbor(self, node: (int, int)):
        """
        :param node:
        :return:
        """
        cost = partial(self.lookahead_cost, node)
        return min(self.est_local_map.neighbors(node), key=cost)

    def calculate_rhs(self, node: (int, int)):
        """
        :param node:
        :return:
        """
        lowest_cost_neighbor = self.lowest_cost_neighbor(node)
        self.back_pointers[node] = lowest_cost_neighbor
        return self.lookahead_cost(node, lowest_cost_neighbor)

    def compute_shortest_path(self):
        last_nodes = deque(maxlen=10)
        while self.U.first_key() < self.calculate_key(self.position) or self.rhs(self.position) != self.g(self.position):
