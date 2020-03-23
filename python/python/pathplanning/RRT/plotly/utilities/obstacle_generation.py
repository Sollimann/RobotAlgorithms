import uuid
import numpy as np

"""
Universally unique identifies (UUID) values are 128 bits long and “can guarantee uniqueness across space and time”.
They are useful for situations where a unique identifiers value is necessary.
"""


def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    :param obstacles:
    :return yield:
    """
    for obstacle in obstacles:
        yield uuid.uuid4(), obstacle, obstacle


def bbox(img):
    img = (img > 0)
    rows = np.any(img, axis=1)
    cols = np.any(img, axis=0)
    rmin, rmax = np.argmax(rows), img.shape[0] - 1 - np.argmax(np.flipud(rows))
    cmin, cmax = np.argmax(cols), img.shape[1] - 1 - np.argmax(np.flipud(cols))
    return rmin, cmin, rmax, cmax


# Python program to find all
# rectangles filled with 0

def findend(i, j, a, output, index):
    x = len(a)
    y = len(a[0])

    # flag to check column edge case,
    # initializing with 0
    flagc = 1

    # flag to check row edge case,
    # initializing with 0
    flagr = 1

    for m in range(i, x):

        # loop breaks where first 1 encounters
        if a[m][j] == 0:
            flagr = 0  # set the flag
            break

        for n in range(j, y):

            # loop breaks where first 1 encounters
            if a[m][n] == 0:
                flagc = 0  # set the flag
                break

    if flagr == 0:
        output[index].append(m - 1)
    else:
        # when end point touch the boundary
        output[index].append(m)

    if flagc == 0:
        output[index].append(n - 1)
    else:
        # when end point touch the boundary
        output[index].append(n)


def get_rectangle_coordinates(a):
    # retrieving the column size of array
    size_of_array = len(a)

    # output array where we are going
    # to store our output
    output = []

    # It will be used for storing start
    # and end location in the same index
    index = -1

    for i in range(0, size_of_array):
        for j in range(0, len(a[0])):
            if a[i][j] == 1:
                # storing initial position
                # of rectangle
                output.append([i, j])

                # will be used for the
                # last position
                index = index + 1
                findend(i, j, a, output, index)

    # Always make obstacles a bit larger than what they actually are
    obstacles = []
    for ob in output:
        ob[0] -= 1
        ob[2] += 1
        ob[1] -= 1
        ob[3] += 1
        obstacles.append(ob)

    return obstacles
