from python.pathplanning.grid_map_astar.utilities.gui import set_obstacle, draw_background, GUI
from python.pathplanning.grid_map_astar.Astar.astar_potential_field import AstarPotentialField
import numpy as np
import traceback
import scipy.ndimage

# The world extents in units.
map_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
# Potential field = any value between 1 and 254
occupancy_grid_map = np.zeros(map_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True


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
    occupancy_grid_map = np.zeros(map_extents, dtype=np.uint8)
    update_callback()


def toggle_visited_nodes():
    global show_visited_nodes
    show_visited_nodes = not show_visited_nodes
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


def toggle_potential_function():
    global use_potential_function
    use_potential_function = not use_potential_function
    update_callback()


def update_callback(pos=None):
    # First apply distance transform to occupancy_grid_map.

    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start == None or goal == None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = plan.ogrid_cb(start, goal, occupancy_grid_map)
        except Exception as e:
            print(traceback.print_exc())
    # Draw new background.
    draw_background(gui, occupancy_grid_map, visited_nodes, optimal_path,
                    show_visited_nodes)


# Main program.
if __name__ == '__main__':
    plan = AstarPotentialField(exploration_setting='8N', use_potential_field=True)

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
               ("Show Visited", toggle_visited_nodes)]

    # Init GUI.
    gui = GUI(map_extents, 4, callbacks,
              buttons, "on",
              "Astar Algorithm with potential field")

    # Start GUI main loop.
    gui.run()
