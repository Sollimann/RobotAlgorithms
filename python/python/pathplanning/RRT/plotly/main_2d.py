import numpy as np

from python.pathplanning.RRT.plotly.rrt.rrt_star import RRTStar
from python.pathplanning.RRT.plotly.search_space.search_space import SearchSpace
from python.pathplanning.RRT.plotly.utilities.plotting import Plot


def main():
    X_dimensions = np.array([(0, 100), (0, 100)])  # dimensions of search space
    Obstacles = np.array([(20, 20, 40, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)])  # [(pos, size), ...]
    x_init = (0, 0)  # starting location
    x_goal = (100, 100)  # goal location

    Q = np.array([(8, 4)])  # length of tree edges
    r = 1  # length of smallest edge to check for intersection with obstacles
    max_samples = 4024  # max number of samples to take before timing out
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
