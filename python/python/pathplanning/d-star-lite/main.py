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
    start = (10, 10)
    goal = (20, 70)

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=5)

    ground_truth_world = gui.world

    new_position = np.array([start[0], start[1]])
    last_position = new_position
    new_observation = None
    type = OBSTACLE

    dstar = Dstar_lite(map=ground_truth_world,
                       s_start=start,
                       s_goal=goal,
                       view_range=5)

    path = dstar.move_and_replan(robot_position=new_position)

    while not gui.done:
        gui.run_game(path=path)

        (x, y) = gui.current
        new_position = np.array([x, y])
        new_observation = gui.observation

        if new_observation is not None:
            if new_observation["type"] == OBSTACLE:
                dstar.global_map.set_obstacle(pos=new_observation["pos"])
            if new_observation["pos"] == UNOCCUPIED:
                print("else {}".format(new_observation["pos"]))
                dstar.global_map.remove_obstacle(pos=new_observation["pos"])

        if new_position != last_position:
            last_position = new_position
            path = dstar.move_and_replan(robot_position=new_position)
