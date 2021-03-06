from python.pathplanning.grid_map_astar.utilities import gui
from python.pathplanning.grid_map_astar.Astar.heuristics import distance, get_movements_4n, get_movements_8n, \
    is_outside_grid
import traceback
import numpy as np
import scipy.ndimage
from heapq import heappush, heappop


class AstarPotentialField:

    def __init__(self, use_potential_field=True, exploration_setting='8N'):
        """
        :param use_potential_field: 'True' if you want repulsive potential field around obstacles
        :param exploration_setting: explore with 4-connectivity movements or 8-connectivity movements
        """
        self.use_potential_field = use_potential_field
        self.exploration_setting = exploration_setting

    def ogrid_cb(self, start: (int, int), goal: (int, int), map):
        """
        A callback function that gets the latest status of the navigation,
        and returns an optimal path using potential field Astar search algorithm
        :param start: (int, int) the node at which the search starts
        :param goal: (int, int) the node at which the search ends
        :param map: A 2D occupancy grid map with traversal space (0) and non traversal space (255)
        :return: A optimal path [(x,y),...] and the global cost map [(x,y, cost),...]
        """
        optimal_path, global_cost_map = AstarPotentialField.astar(start=start, goal=goal, occupancy_grid_map=map,
                                                                  exploration_setting=self.exploration_setting,
                                                                  use_potential_field=self.use_potential_field)

        return optimal_path, global_cost_map

    @staticmethod
    def apply_obstacle_potential_field(map, potential_field: bool):
        if potential_field and np.max(map) == 255:
            # compute distance transform
            dist_transform = 255 - np.minimum(16 * scipy.ndimage.morphology.distance_transform_edt(255 - map), 255)
            m = max(np.max(dist_transform), 1)  # prevent m == 0
            potential_field_map = np.uint8((dist_transform * 255) / m)
            return potential_field_map
        else:
            # keep 255 values only, and set all other to 0
            potential_field_map = (map == 255) * np.uint8(255)
            return potential_field_map

    @staticmethod
    def astar(start: (int, int), goal: (int, int), occupancy_grid_map,
              exploration_setting='8N', use_potential_field=True):
        """
        :param occupancy_grid_map: A 2D occupancy grid map with traversal space (0) and non traversal space (255)
        :param use_potential_field: 'True' if you want repulsive potential field around obstacles
        :param exploration_setting: explore with 4-connectivity movements or 8-connectivity movements
        :param start: (int, int) the node at which the search starts
        :param goal: (int, int) the node at which the search ends
        :return: A optimal path [(x,y),...] and the global cost map [(x,y, cost),...]
        """

        # the first element is the total cost astar = dijkstra optimal to a point + greedy estimate to goal
        # the second element is the cost of the path from the start to the point
        # the third element is the position (cell / node) of the point
        # the fourth component is the position we came from when entering the (initialized as None)
        stack = [(0.001 + distance(start, goal), 0.001, start, None)]

        # If use_potential_field is True, at repulsive potential field to occupied cells
        occupancy_grid_map = AstarPotentialField.apply_obstacle_potential_field(map=occupancy_grid_map,
                                                                                potential_field=use_potential_field)

        extents = occupancy_grid_map.shape
        global_cost_map = np.zeros(extents, dtype=np.float32)  # 32 bit float for cost

        # Also, we use a dictionary to remember where we came from
        reconstructed_path = {}

        while stack:
            # retrieve the smallest cost item and remove from stack
            total_cost, cost, pos, previous = heappop(stack)

            # IMPORTANT: must make sure that the indices are integers
            pos = (round(pos[0]), round(pos[1]))

            # if node has already been visited, the proceed to next element
            # in the stack
            if global_cost_map[pos] > 0:
                continue

            # now that the node has been visited. Mark with cost
            # also mark the node we came from
            global_cost_map[pos] = cost
            reconstructed_path[pos] = previous

            # check if goal has yet been reached
            if pos == goal:
                break  # finished!

            new_x = None
            new_y = None

            # check all neighboring nodes
            if exploration_setting == '4N':
                movements = get_movements_4n()
            else:
                movements = get_movements_8n()

            for dx, dy, deltacost in movements:
                # determine new position and check bounds
                new_x = pos[0] + dx
                new_y = pos[1] + dy

                # if the explored node is outside of grid, skip the remaining part of this loop iteration
                if is_outside_grid(x=new_x, y=new_y, x_lim=extents[0], y_lim=extents[1]):
                    continue

                # else, proceed with new pos
                new_pos = (new_x, new_y)

                # if visited is 0 (unexplored) and it does not hit cost 255 (obstacle), then:
                # append the tuple to the stack, so we can explore this nodes neighbor in next iteration
                if not global_cost_map[new_pos] and occupancy_grid_map[new_pos] != 255:
                    new_cost = cost + deltacost + occupancy_grid_map[new_pos] / 64.0
                    new_total_cost = new_cost + distance(new_pos, goal)

                    # heappush() will insert in the at its right place in the sorted stack
                    # so that we can explore its neighbors in next iteration
                    heappush(stack, (new_total_cost, new_cost, new_pos, pos))

        # ones we are done exploring, reconstruct path if the goal node has been reached
        # else, return an empty path
        path = []
        if pos == goal:  # if goal is reached, unwind backwards
            while pos:
                path.append(pos)
                pos = reconstructed_path[pos]
            path.reverse()  # reverse so that path is from start to goal
        return path, global_cost_map
