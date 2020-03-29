
from python.pathplanning.grid_map_astar.Astar.heuristics import distance, get_movements_4n, get_movements_8n, \
    is_outside_grid
import traceback
import numpy as np
import scipy.ndimage
from heapq import heappush, heappop


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


def astar(start: (int, int), goal: (int, int), occupancy_grid_map,
          exploration_setting='8N', use_potential_field=True):
    """
    :param occupancy_grid_map:
    :param use_potential_field:
    :param exploration_setting:
    :param start:
    :param goal:
    :return:
    """

    # the first element is the total cost astar = dijkstra optimal to a point + greedy estimate to goal
    # the second element is the cost of the path from the start to the point
    # the third element is the position (cell / node) of the point
    # the fourth component is the position we came from when entering the (initialized as None)
    stack = [(0.001 + distance(start, goal), 0.001, start, None)]

    # in the beginning, no cell has been visited
    occupancy_grid_map = apply_obstacle_potential_field(map=occupancy_grid_map,
                                                        potential_field=use_potential_field)

    extents = occupancy_grid_map.shape
    visited = np.zeros(extents, dtype=np.float32)  # 32 bit float for cost

    # Also, we use a dictionary to remember where we came from
    reconstructed_path = {}

    while stack:
        # retrieve the smallest cost item and remove from stack
        total_cost, cost, pos, previous = heappop(stack)

        # IMPORTANT: must make sure that the indices are integers
        pos = (round(pos[0]), round(pos[1]))

        # if node has already been visited, the proceed to next element
        # in the stack
        if visited[pos] > 0:
            continue

        # now that the node has been visited. Mark with cost
        # also mark the node we came from
        visited[pos] = cost
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
            if not visited[new_pos] and occupancy_grid_map[new_pos] != 255:
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

    return path, visited


if __name__ == '__main__':
    pass
