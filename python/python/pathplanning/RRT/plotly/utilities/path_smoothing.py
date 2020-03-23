from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np


def main():
    path = [(0, 0), (16.592219301553833, 43.79725391339021), (37.27171177167165, 56.83003857575241),
            (56.673551532106266, 75.43448682921361), (56.421628968575604, 83.4305192842493), (100, 100)]

    x = [o[0] for o in path]
    y = [o[1] for o in path]
    print(x)


if __name__ == "__main__":
    main()
