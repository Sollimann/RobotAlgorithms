from gui import *
from dstar_lite import *
from grid import *

if __name__ == '__main__':

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 100
    y_dim = 80
    start = (1, 1)
    goal = (90, 70)

    estimated_world_occupancy_grid = OccupancyGridMap(x_dim=x_dim,
                                                y_dim=y_dim,
                                                start_x=start[0],
                                                start_y=start[1],
                                                goal_x=goal[0],
                                                goal_y=goal[1])

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    viewing_range=2)

    ground_truth_world_occupancy_grid = gui.world.occupancy_grid_map

    dstar = DstarLite(graph=estimated_world_occupancy_grid,
                      start=start,
                      goal=goal,
                      view_range=2)

    while not gui.done:
        gui.run_game()