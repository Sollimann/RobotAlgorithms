from python.pathplanning.grid_map_astar.utilities.gui import set_obstacle, draw_background, GUI
from python.pathplanning.grid_map_astar.Astar.astar_potential_field import AstarPotentialField
import numpy as np
import traceback
import scipy.ndimage

# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
# Potential field = any value between 1 and 254
occupancy_grid_map = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

plan = AstarPotentialField()


# Functions for GUI functionality.
def add_obstacle(pos):
    set_obstacle(occupancy_grid_map, pos, True)
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


def remove_obstacle(pos):
    set_obstacle(occupancy_grid_map, pos, False)
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


def clear_obstacles():
    global occupancy_grid_map
    occupancy_grid_map = np.zeros(world_extents, dtype=np.uint8)
    update_callback()


"""
     Potential field specific
"""
# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True


def toggle_visited_nodes():
    global show_visited_nodes
    show_visited_nodes = not show_visited_nodes
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


def toggle_potential_function():
    global use_potential_function
    use_potential_function = not use_potential_function
    update_callback()


def apply_distance_transform():
    global occupancy_grid_map
    if use_potential_function and np.max(occupancy_grid_map) == 255:
        # Compute distance transform.
        dist_transform = 255 - np.minimum(
            16 * scipy.ndimage.morphology.distance_transform_edt(
                255 - occupancy_grid_map), 255)
        m = max(np.max(dist_transform), 1)  # Prevent m==0.
        occupancy_grid_map = np.uint8((dist_transform * 255) / m)
    else:
        # Keep 255 values only (set all other to 0).
        occupancy_grid_map = (occupancy_grid_map == 255) * np.uint8(255)


def update_callback(pos=None):
    # First apply distance transform to occupancy_grid_map.
    apply_distance_transform()  # potential field
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start == None or goal == None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = plan.astar(start=start, goal=goal, occupancy_grid_map=occupancy_grid_map,
                                                     exploration_setting='8N', use_potential_field=True)
        except Exception as e:
            print(traceback.print_exc())
    # Draw new background.
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


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
               ("Use Potential Function", toggle_potential_function),
               ("Show Visited", toggle_visited_nodes)]

    # Init GUI.
    gui = GUI(world_extents, 4, callbacks,
              buttons, "on",
              "Simple Dijkstra Algorithm (finally shows the optimal path "
              "from start to goal).")

    # Start GUI main loop.
    gui.run()
