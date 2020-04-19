from collections import deque
from functools import partial
from typing import Set

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
        self.position = s_start
        self.goal = s_goal
        self.back_pointers = {}

        # procedure initialize
        self.U = PriorityQueue()
        self.k_m = 0
        self.RHS_VALS = {}
        self.G_VALS = {}  # empty set
        self.RHS_VALS[s_goal] = 0.0
        self.U.insert(s_goal, self.calculate_key(s_goal))

        # keeps track of the best path starting from goal to our position
        self.back_pointers[self.goal] = None

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
        return self.RHS_VALS.get(node, float('inf')) if node != self.goal else 0

    def calculate_key(self, node: (int, int)):
        """
        procedure CalculateKey(s)
        :param node:
        :return:
        """
        g_rhs = min([self.g(node), self.rhs(node)])
        #return g_rhs + heuristic(node, self.position), g_rhs
        return g_rhs + self.est_global_map.cost(node, self.position) + self.k_m, g_rhs

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
            self.U.insert(node, self.calculate_key(node))

    def update_vertices(self, nodes):
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
        return self.g(neighbor) + self.est_global_map.cost(neighbor, node)

    def lowest_cost_neighbor(self, node: (int, int)) -> (int, int):
        """
        :param node:
        :return:
        """
        cost = partial(self.lookahead_cost, node)
        best_choice = min(self.est_global_map.neighbors(node), key=cost)
        """
        if len(best_choice) == 0:
            print("no choice")
            self.est_global_map.visited = set(self.position)
            return min(self.est_global_map.neighbors(node), key=cost)
        """
        #print("best choice: {}".format(best_choice))
        #if best_choice not in self.est_global_map.visited:
            #self.est_global_map.visited.add(best_choice)
            #best_choice = min(self.est_global_map.neighbors(node), key=cost)
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
        s_start = self.position

        while self.U.top_key(s_start) < self.calculate_key(s_start) or self.rhs(s_start) != self.g(s_start):
            u = self.U.pop_top()
            k_old = self.U.top_key(s_start)

            # NOT PART OF ALGORITHM!
            last_nodes.append(u)
            if len(last_nodes) == 10 and len(set(last_nodes)) < 3:
                raise Exception("Failed! Stuck in a loop")
            # NOT PART OF ALGORITHM!

            k_new = self.calculate_key(u)
            if k_old < k_new:
                self.U.insert(u, k_new)  # Essentially update, since u is already popped, and we put it back again
            elif self.g(node=u) > self.rhs(node=u):
                self.G_VALS[u] = self.rhs(node=u)

                # since u is already removed, we update vertices
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
        self.position = position
        # rescan local area
        local_observation = self.gt_global_map.local_observation(global_position=self.robot_position,
                                                                 view_range=self.view_range)
        # update global map from local data
        # return new obstacles added to the map
        cells_with_new_cost = self.est_global_map.update_global_from_local_grid(local_grid=local_observation)

        # replan
        self.compute_shortest_path()

        # make current position as last node
        s_start = self.position
        s_last = s_start

        yield s_last, cells_with_new_cost

        # while we yet haven't reached the goal
        count = 0
        self.est_global_map.visited = {(1, 1)}
        while s_start != self.goal:
            #print("s_start: {}, position: {}".format(s_start, self.robot_position))
            if self.rhs(s_start) == float('inf'):
                raise Exception("No path found!")

            # find next lowest cost neighbor
            s_start = self.lowest_cost_neighbor(s_start)
            #print("visited: {}".format(self.est_global_map.visited))
            self.est_global_map.visited.add(s_start)


            # move to next lowest cost neighbor. This might take a while
            # in real life implementation
            self.position = s_start

            # scan graph for changed edge costs
            local_observation = self.est_global_map.local_observation(global_position=self.position,
                                                                      view_range=self.view_range)

            # FIX NEEDED. This should not return cells_with_new_cost if there not in local view
            cells_with_new_cost = self.est_global_map.update_global_from_local_grid(local_grid=local_observation)

            # if any edge cost changed
            if cells_with_new_cost:
                #print("cells with new cost {}".format(cells_with_new_cost))
                self.k_m += self.est_global_map.cost(s_last, s_start)  # update heuristics
                s_last = s_start
                self.update_vertices({node for cell in cells_with_new_cost
                                      for node in self.est_global_map.neighbors(cell)
                                      if self.est_global_map.is_unoccupied(node)})  # this last line is not needed
                self.compute_shortest_path()
            yield s_start, cells_with_new_cost

        print("goal found!")
