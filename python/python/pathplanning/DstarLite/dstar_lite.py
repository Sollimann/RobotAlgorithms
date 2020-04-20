from collections import deque
from functools import partial
import heapq

from utils import *
from priority_queue import PriorityQueue
from grid import OccupancyGridMap


class DstarLite(object):
    def __init__(self, world, s_start: (int, int), s_goal: (int, int), view_range=2):
        # init the graphs
        self.est_global_map = OccupancyGridMap(x_dim=world.x_dim,
                                               y_dim=world.y_dim,
                                               exploration_setting='8N')

        # real_graph
        self.gt_global_map: OccupancyGridMap = world

        # init variables
        self.view_range = view_range
        self.s_start = s_start
        self.s_goal = s_goal
        self.back_pointers = {}

        # procedure initialize
        self.U = PriorityQueue()
        self.k_m = 0
        self.RHS_VALS = {}
        self.G_VALS = {}  # empty set
        self.RHS_VALS[s_goal] = 0.0
        #self.U.insert(s_goal, self.calculate_key(s_goal))
        self.U.insert(s_goal, (heuristic(s_start, s_goal), 0))

        # keeps track of the best path starting from goal to our position
        self.back_pointers[self.s_goal] = None

        # replan
        self.compute_shortest_path()

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
        # if node == self.position:
        #    return 0
        return self.RHS_VALS.get(node, float('inf')) if node != self.s_goal else 0

    def calculate_key(self, s: (int, int)):
        """
        procedure CalculateKey(s)
        :param s:
        :return:
        """
        k1 = min([self.g(s), self.rhs(s)]) + heuristic(self.s_start, s) + self.k_m
        k2 = min([self.g(s), self.rhs(s)])
        return k1, k2

    def update_vertex(self, u: (int, int)):
        """
        procedure UpdateVertex(u)
        :param u:
        :return:
        """
        g_u = self.g(node=u)
        rhs_u = self.rhs(node=u)
        u_in_queue = self.U.is_in_queue(id=u)

        if g_u != rhs_u and u_in_queue:
            self.U.update(id=u,priority=self.calculate_key(u))
        elif g_u != rhs_u and not u_in_queue:
            self.U.insert(item=u,priority=self.calculate_key(u))
        elif g_u == rhs_u and u_in_queue:
            self.U.remove(id=u)

    def update_vertices(self, nodes):
        """
        :param nodes:
        :return:
        """
        for node in nodes:
            self.update_vertex(u=node)

    def lookahead_cost(self, node, neighbor):
        """
        :param node:
        :param neighbor:
        :return:
        """
        return self.g(neighbor) + self.est_global_map.cost(neighbor, node)

    def lowest_cost_neighbor(self, node: (int, int)) -> (int, int):
        """
        :param node:
        :return:
        """
        cost = partial(self.lookahead_cost, node)
        best_choice = min(self.est_global_map.neighbors(node), key=cost)
        return best_choice

    def calculate_rhs(self, node: (int, int)) -> float:
        """
        :param node:
        :return:
        """
        lowest_cost_neighbor = self.lowest_cost_neighbor(node)
        self.back_pointers[node] = lowest_cost_neighbor
        return self.lookahead_cost(node, lowest_cost_neighbor)

    def compute_shortest_path(self):
        """
        Procedure ComputeShortestPath()
        :return:
        """
        last_nodes = deque(maxlen=10)

        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs(self.s_start) != self.g(self.s_start):
            u = self.U.top()
            k_old = self.U.top_key()

            # NOT PART OF ALGORITHM!(s_start=s_start
            last_nodes.append(u)
            if len(last_nodes) == 10 and len(set(last_nodes)) < 3:
                raise Exception("Failed! Stuck in a loop")
            # NOT PART OF ALGORITHM!

            k_new = self.calculate_key(s=u)
            if k_old < k_new:
                self.U.update(id=u, priority=k_new)  # Essentially update, since u is already popped, and we put it back again
            elif self.g(node=u) > self.rhs(node=u):
                self.G_VALS[u] = self.rhs(node=u)
                self.U.remove(id=u)
                self.update_vertices(nodes=self.est_global_map.neighbors(u))
            else:
                self.G_VALS[u] = float('inf')
                self.update_vertices(nodes=self.est_global_map.neighbors(u) + [u])  # list + list = list

        return self.back_pointers.copy(), self.G_VALS.copy()

    def move_and_rescan(self, position: (int, int)):
        """
        Procedure Main()
        :return:
        """
        self.robot_position = position
        self.s_start = position
        # rescan local area
        local_observation = self.gt_global_map.local_observation(global_position=self.robot_position,
                                                                 view_range=self.view_range)
        # update global map from local data
        # return new obstacles added to the map
        cells_with_new_cost = self.est_global_map.update_global_from_local_grid(local_grid=local_observation)

        # make current position as last node
        s_last = self.s_start

        yield s_last, cells_with_new_cost

        # while we yet haven't reached the goal
        #visited = {s_start}
        while self.s_start != self.s_goal:
            # print("s_start: {}, position: {}".format(s_start, self.robot_position))
            if self.rhs(self.s_start) == float('inf'):
                raise Exception("No path found!")

            # find next lowest cost neighbor
            self.s_start = self.lowest_cost_neighbor(self.s_start)
            # move to next lowest cost neighbor. This might take a while
            #self.position = s_start

            # scan graph for changed edge costs
            local_observation = self.gt_global_map.local_observation(global_position=self.s_start,
                                                                     view_range=self.view_range)

            # FIX NEEDED. This should not return cells_with_new_cost if there not in local view
            cells_with_new_cost = self.est_global_map.update_global_from_local_grid(local_grid=local_observation)

            # if any edge cost changed
            if cells_with_new_cost:
                # print("cells with new cost {}".format(cells_with_new_cost))
                self.k_m += self.est_global_map.cost(s_last, self.s_start)  # update heuristics
                s_last = self.s_start
                self.update_vertices({node for cell in cells_with_new_cost
                                      for node in self.est_global_map.neighbors(cell)
                                      if self.est_global_map.is_unoccupied(node)})  # this last line is not needed
                self.compute_shortest_path()
            yield self.s_start, cells_with_new_cost

        print("goal found!")
