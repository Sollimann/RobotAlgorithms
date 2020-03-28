import numpy as np
from python.pathplanning.RRT.plotly.rrt.rrt_star import RRTStar
from python.pathplanning.RRT.plotly.search_space.search_space import SearchSpace
from python.pathplanning.RRT.plotly.utilities.plotting import Plot
from python.pathplanning.RRT.plotly.utilities.obstacle_generation import get_rectangle_coordinates


def main():
    X_dimensions = np.array([(0, 20), (0, 20)])  # dimensions of search space

    # driver code
    ogrid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, -1, -1, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, -1, -1, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, -1, -1, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0, 0, 100, 100, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
        [0, 0, 0, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, -1, -1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, -1, -1, -1, -1, 0, 0, 0],
        [0, 0, 100, 100, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, -1, -1, -1, 0, 0, 0],
        [0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    ogrid_threshold = float("90")
    occ_img = 1*np.greater(ogrid, ogrid_threshold).astype(np.uint8)
    print(occ_img)
    obs = get_rectangle_coordinates(occ_img)
    Obstacles = np.array(obs)  # [(from_x, from_y, to_x, to_y), ...]
    x_init = (0, 0)  # starting location
    x_goal = (19, 19)  # goal location

    Q = np.array([(8, 4)])  # length of tree edges
    r = 1  # length of smallest edge to check for intersection with obstacles
    max_samples = 2024  # max number of samples to take before timing out
    rewire_count = 32  # optional, number of nearby branches to rewire
    prc = 0.1  # probability of checking for a connection to goal

    # Create Search Space
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt* search
    rrt = RRTStar(X=X, Q=Q, x_init=x_init, x_goal=x_goal,
                  max_samples=max_samples, r=r, prc=prc, rewire_count=rewire_count)
    path = rrt.rrt_star()
    print(path)

    # plot
    plot = Plot(filename="rrt_star_2d")
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)


if __name__ == "__main__":
    main()
