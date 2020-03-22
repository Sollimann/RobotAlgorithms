import uuid

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
