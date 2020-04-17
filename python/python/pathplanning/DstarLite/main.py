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
    goal = (50, 50)

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    viewing_range=2)

    ground_truth_world = gui.world

    dstar = DstarLite(world=ground_truth_world,
                      s_start=start,
                      s_goal=goal,
                      view_range=2)

    while not gui.done:

        # update the map


        # compute new path
        path = [p for p, o in dstar.move_and_rescan()]

        # drive gui
        gui.run_game(path=path)
