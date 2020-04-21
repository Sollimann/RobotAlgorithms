import heapq
import numpy as np
from utils import *
# from priority_queue import PriorityQueue
from grid import OccupancyGridMap


class DstarLite(object):
    def __init__(self, map, s_start: (int, int), s_goal: (int, int), view_range=2):
        self.view_range = view_range
        self.s_start = s_start
        self.s_goal = s_goal
        self.k_m = 0
        self.rhs = np.ones((map.x_dim, map.y_dim)) * np.inf
        self.g = self.rhs.copy()
        self.global_map = map
        self.sensed_map = OccupancyGridMap(x_dim=map.x_dim,
                                           y_dim=map.y_dim,
                                           exploration_setting='8N')
        self.rhs[self.s_goal] = 0
        self.U = []
        A = Element(self.s_goal, *self.calculate_key(self.s_goal))
        heapq.heappush(self.U, A)

    def c(self, u: (int, int), v: (int, int)) -> float:
        """
        calcuclate the cost between nodes
        :param u:
        :param v:
        :return:
        """
        if not self.sensed_map.is_unoccupied(u) or not self.sensed_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def calculate_key(self, s: (int, int)):
        k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return k1, k2

    def update_vertex(self, u):
        if u != self.s_goal:
            succ = self.sensed_map.succ(cell=u)
            min_s = float('inf')
            for s in succ:
                if self.c(u, s) + self.g[s] < min_s:
                    min_s = self.c(u, s) + self.g[s]
            self.rhs[u] = min_s
        if Element(u, 0, 0) in self.U:
            self.U.remove(Element(u, 0, 0))
            heapq.heapify(self.U)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, Element(u, *self.calculate_key(u)))

    def compute_shortest_path(self):
        # should fix here. not right algorithm
        while heapq.nsmallest(1, self.U)[0] < Element(self.s_start, *self.calculate_key(self.s_start))\
                or self.rhs[self.s_start] != self.g[self.s_start]:

            k_old = heapq.nsmallest(1, self.U)[0]
            u = heapq.heappop(self.U).key
            k_new = Element(u, *self.calculate_key(u))
            if k_old < k_new:
                heapq.heappush(self.U, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                pred = self.sensed_map.succ(cell=u)
                for s in pred:
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                pred = self.sensed_map.succ(cell=u)
                pred.append(u)
                for s in pred:
                    self.update_vertex(s)

    def rescan(self, global_position: (int, int), gt: bool):

        if gt:
            # rescan local area
            local_observation = self.global_map.local_observation(global_position=global_position,
                                                                  view_range=self.view_range)
        else:
            # rescan local area
            local_observation = self.sensed_map.local_observation(global_position=global_position,
                                                                  view_range=self.view_range)

        #for key, value in local_observation.items():
        #    if value == 255:
        #        print(key)
        # update global map from local data
        # return new obstacles added to the map
        cells_with_new_cost = self.sensed_map.update_global_from_local_grid(local_grid=local_observation)
        #print("cells with new cost: {}".format(cells_with_new_cost))
        return cells_with_new_cost

    def move_and_replan(self, robot_position: (int, int)):
        path = [robot_position]
        self.s_start = robot_position
        s_last = self.s_start
        #print("move")
        # rescan
        self.rescan(global_position=self.s_start, gt=True)

        # recompute path
        self.compute_shortest_path()

        count = 0
        visited = {self}
        while self.s_start != self.s_goal:
            if self.g[self.s_start] == float('inf'):
                print(self.g)
                raise Exception("No path found!")
            count += 1
            #print("while")
            succ = self.sensed_map.succ(cell=self.s_start)
            min_s = float('inf')
            for s in succ:
                if self.c(self.s_start, s) + self.g[s] < min_s:
                    min_s = self.c(self.s_start, s) + self.g[s]
                    temp = s
            # move to next
            #print("s_start: {}".format(self.s_start))
            self.s_start = temp
            path.append(self.s_start)
            print(count)

            # scan graph for changed costs
            cells_with_new_cost = self.rescan(global_position=self.s_start, gt=False)

            # if any edge costs changed
            if cells_with_new_cost:
                self.k_m += heuristic(s_last, self.s_start)
                s_last = self.s_start
                for c in cells_with_new_cost:
                    succ = self.sensed_map.succ(c)
                    for u in succ:
                        if self.sensed_map.is_unoccupied(u):
                            self.update_vertex(u)
                self.compute_shortest_path()

        print("path found!")
        return path, self.sensed_map.occupancy_grid_map
