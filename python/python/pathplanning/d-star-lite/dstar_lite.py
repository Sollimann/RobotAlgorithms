from utils import Element, heuristic, get_movements_4n, get_movements_8n
import numpy as np
from grid import OccupancyGridMap
import heapq


class Dstar_lite:

    def __init__(self, map, s_start: (int, int), s_goal: (int, int), view_range: int):
        """
        :param world:
        :param s_start:
        :param s_goal:
        :param view_range:
        """
        self.view_range = view_range
        self.s_start = np.array([s_start[0], s_start[1]])
        self.s_goal = np.array([s_goal[0], s_goal[1]])
        self.k_m = 0
        self.rhs = np.ones((map.x_dim, map.y_dim)) * np.inf
        self.g = self.rhs.copy()
        self.global_map = map
        self.sensed_map = OccupancyGridMap(x_dim=map.x_dim,
                                           y_dim=map.y_dim,
                                           exploration_setting='8N')
        print(self.global_map)
        print(self.sensed_map)

        self.rhs[self.s_goal[0], self.s_goal[1]] = 0
        self.U = []
        goal = Element(self.s_goal, *self.calculate_key(self.s_goal))
        heapq.heappush(self.U, goal)

    def calculate_key(self, s):
        #print("calc key: {}".format(s))
        key = [0, 0]
        key[0] = min(self.g[s[0], s[1]], self.rhs[s[0], s[1]]) + heuristic(self.s_start, s) + self.k_m
        key[1] = min(self.g[s[0], s[1]], self.rhs[s[0], s[1]])
        return key

    def c(self, u, v) -> float:
        """
        calcuclate the cost between nodes
        :param u:
        :param v:
        :return:
        """
        if not self.sensed_map.is_unoccupied((u[0], u[1])) or not self.sensed_map.is_unoccupied((v[0], v[1])):
            return np.inf
        else:
            return heuristic(u, v)

    def update_vertex(self, u):
        #print("update vertex u: {}".format(u))
        if np.sum(np.abs(u - self.s_goal)) != 0:
            succ = self.sensed_map.succ(cell=u)
            min_s = np.inf
            for s in succ:
                if self.c(u, s) + self.g[s[0], s[1]] < min_s:
                    min_s = self.c(u, s) + self.g[s[0], s[1]]
            self.rhs[u[0], u[1]] = min_s
        if Element(u, 0, 0) in self.U:
            self.U.remove(Element(u, 0, 0))
            heapq.heapify(self.U)
        if self.g[u[0], u[1]] != self.rhs[u[0], u[1]]:
            heapq.heappush(self.U, Element(u, *self.calculate_key(u)))

    def compute_shortest_path(self):
        while len(self.U) > 0 and heapq.nsmallest(1, self.U)[0] < Element(self.s_start,
                                                                          *self.calculate_key(self.s_start)):
            k_old = heapq.nsmallest(1, self.U)[0]
            u = heapq.heappop(self.U).key
            k_new = Element(u, *self.calculate_key(u))
            if k_old < k_new:
                heapq.heappush(self.U, k_new)
            elif self.g[u[0], u[1]] > self.rhs[u[0], u[1]]:
                self.g[u[0], u[1]] = self.rhs[u[0], u[1]]
                pred = self.sensed_map.succ(cell=u)
                for s in pred:
                    self.update_vertex(s)
            else:
                self.g[u[0], u[1]] = np.inf
                pred = self.sensed_map.succ(cell=u)
                pred.append(u)
                for s in pred:
                    self.update_vertex(s)

    def rescan(self, global_position):
        # rescan local area
        local_observation = self.global_map.local_observation(global_position=global_position,
                                                              view_range=self.view_range)
        # update global map from local data
        # return new obstacles added to the map
        cells_with_new_cost = self.sensed_map.update_global_from_local_grid(local_grid=local_observation)
        #print("cells with new cost: {}".format(cells_with_new_cost))
        return cells_with_new_cost

    def move_and_replan(self, robot_position):
        path = [robot_position]

        self.s_start = robot_position
        s_last = self.s_start
        self.rescan(global_position=robot_position)
        self.compute_shortest_path()

        while np.sum(np.abs(self.s_start - self.s_goal)) != 0:
            if self.g[self.s_start[0], self.s_start[1]] == np.inf:
                raise Exception("No path found!")

            succ = self.sensed_map.succ(cell=self.s_start)
            min_s = np.inf
            for s in succ:
                if self.c(self.s_start, s) + self.g[s[0], s[1]] < min_s:
                    min_s = self.c(self.s_start, s) + self.g[s[0], s[1]]
                    temp = s
            # move to next
            self.s_start = temp.copy()
            path.append(self.s_start)

            # scan graph for changed costs
            cells_with_new_cost = self.rescan(global_position=self.s_start)

            # if any edge costs changed
            if cells_with_new_cost:
                self.k_m += heuristic(s_last, self.s_start)
                s_last = self.s_start
                for u in cells_with_new_cost:
                    self.update_vertex(u=u)
                self.compute_shortest_path()
            print(path)
        return path, self.sensed_map.occupancy_grid_map



