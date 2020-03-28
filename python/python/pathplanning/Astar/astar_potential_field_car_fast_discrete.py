from python.pathplanning.Astar import gui
from python.pathplanning.Astar import common
from python.pathplanning.Astar.car_statespace import CurveSegment
import traceback
import numpy as np
import sys
from math import radians, pi, floor
import scipy.ndimage
from heapq import heappush, heappop

# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_cells = None

# Switch which determines if visited cells shall be drawn in the GUI.
show_visited_cells = True

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []


# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)


def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)


def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()


def toggle_movement():
    global max_movement_id
    max_movement_id = 9 - max_movement_id  # Toggles between 3 and 6.
    update_callback()


"""
     Potential field specific
"""
# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True


def toggle_visited_cells():
    global show_visited_nodes
    show_visited_nodes = not show_visited_nodes
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_nodes)


def toggle_potential_function():
    global use_potential_function
    use_potential_function = not use_potential_function
    update_callback()


def apply_distance_transform():
    global world_obstacles
    if use_potential_function and np.max(world_obstacles) == 255:
        # Compute distance transform.
        dist_transform = 255 - np.minimum(
            16 * scipy.ndimage.morphology.distance_transform_edt(
                255 - world_obstacles), 255)
        m = max(np.max(dist_transform), 1)  # Prevent m==0.
        world_obstacles = np.uint8((dist_transform * 255) / m)
    else:
        # Keep 255 values only (set all other to 0).
        world_obstacles = (world_obstacles == 255) * np.uint8(255)


def update_callback(pos=None):
    # First apply distance transform to world_obstacles.
    apply_distance_transform()  # potential field

    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start == None or goal == None):
        global optimal_path
        global visited_cells
        try:
            optimal_path, visited_cells = \
                astar_car_kinematics(start, goal, world_obstacles)
        except Exception as e:
            print(traceback.print_exc())
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)


# --------------------------------------------------------------------------
# Exploration of car's kinematic state space.
# --------------------------------------------------------------------------

# Allowed movements. These are given as tuples: (curvature, length).
movements = [(1.0 / 10, 5.0), (0.0, 5.0), (-1.0 / 10, 5.0),
             (1.0 / 10, -5.0), (0.0, -5.0), (-1.0 / 10, -5.0)]
max_movement_id = 6  # 3 for forward only, 6 for forward and backward.


# Helper functions.
def distance(p, q):
    """Return Euclidean distance between two points."""
    return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def states_close(p, q):
    """Checks if two poses (x, y, heading) are the same within a tolerance."""
    d_angle = abs((p[2] - q[2] + pi) % (2 * pi) - pi)
    # For the sake of simplicity, tolerances are hardcoded here:
    # 15 degrees for the heading angle, 2.0 for the position.
    return d_angle < radians(15.) and distance(p, q) <= 2.0


def pose_index(pose):
    """Given a pose, returns a discrete version (a triple index)."""
    # Again, raster sizes are hardcoded here for simplicity.
    pos_raster = 1.0
    heading_raster = radians(10.)
    xi = int(floor(pose[0] / pos_raster))
    yi = int(floor(pose[1] / pos_raster))
    ti = int(floor(pose[2] / heading_raster))
    return xi, yi, ti


def astar_car_kinematics(start_pose, goal_pose, obstacles):
    """Try to find a sequence of curve segments from start to goal."""

    # Init front: the only element is the start pose.
    # Each tuple contains:
    # (total_cost, cost, pose, previous_pose, move_index).
    front = [(0.0001 + distance(start_pose, goal_pose), 0.0001, start_pose, None, None)]

    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited_cells = np.zeros(extents, dtype=np.float32)

    # Also, no states have been generated.
    generated_states = {}

    while front:
        # Stop search if the front gets too large.
        if len(front) > 1000000:
            print("Timeout.")
            break

        # Pop smallest item from heap.
        total_cost, cost, pose, previous_pose, move = heappop(front)

        # compute discrete index: compute a discrete pose
        pose_idx = pose_index(pose)

        # check if this has been visited already: check if pose_index is in generated states already,
        # and if so, skip the rest of the loop. (this is the point where we prevent exponential growth)
        if pose_idx in generated_states:
            continue

        # Mark visited_cell which encloses this pose.
        visited_cells[int(pose[0]), int(pose[1])] = cost

        # Enter into visited states, also use this to remember where we
        # came from and which move we used.
        generated_states[pose_idx] = (previous_pose, move)

        # Check if we have (approximately) reached the goal.
        if states_close(pose, goal_pose):
            break  # Finished!

        # Check all possible movements.
        for i in range(max_movement_id):
            curvature, length = movements[i]

            # Determine new pose and check bounds.
            new_pose = CurveSegment.end_pose(pose, curvature, length)
            if not (0 <= new_pose[0] < extents[0] and 0 <= new_pose[1] < extents[1]):
                continue

            # Add to front if there is no obstacle.
            if not obstacles[(int(new_pose[0]), int(new_pose[1]))] == 255:
                new_cost = cost + abs(length)
                total_cost = new_cost + distance(new_pose, goal_pose)
                heappush(front, (total_cost, new_cost, new_pose, pose, i))

    # Reconstruct path, starting from goal.
    if states_close(pose, goal_pose):
        path = []
        path.append(pose[0:2])
        pose, move = generated_states[pose_index(pose)]
        while pose:
            points = CurveSegment.segment_points(pose,
                                                 movements[move][0], movements[move][1], 2.0)
            path.extend(reversed(points))
            pose, move = generated_states[pose_index(pose)]
        path.reverse()
    else:
        path = []

    return path, visited_cells


# Main program.
if __name__ == '__main__':
    # Link functions to buttons.
    callbacks = {"update": update_callback,
                 "button_1_press": add_obstacle,
                 "button_1_drag": add_obstacle,
                 "button_1_release": update_callback,
                 "button_2_press": remove_obstacle,
                 "button_2_drag": remove_obstacle,
                 "button_2_release": update_callback,
                 "button_3_press": remove_obstacle,
                 "button_3_drag": remove_obstacle,
                 "button_3_release": update_callback,
                 }

    # Extra buttons.
    buttons = [("Clear", clear_obstacles),
               ("Forward/FW+BW", toggle_movement),
               ("Use Potential Function", toggle_potential_function),
               ("Show Visited", toggle_visited_cells)]

    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "oriented",
                  "Find drivable path by car state space exploration.")

    # Start GUI main loop.
    gui.run()
