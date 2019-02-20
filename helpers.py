import numpy as np


class MagicDict(dict):
    """Content is accessable like property."""

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, deepcopy(v, memo))
        return result

    def __getattr__(self, attr):
        """called what self.attr doesn't exist."""
        return self[attr]


class Colour:
    ALPHA_CK = 255, 0, 255
    white = 255, 255, 255
    black = 20, 20, 40
    red = 255, 0, 0
    blue = 0, 0, 255
    path_blue = 26, 128, 178
    green = 0, 150, 0
    cyan = 20, 200, 200
    orange = 255, 160, 16


class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0  # index 0 is x, index 1 is y
        self.parent = None
        self.children = []


class Stats:
    def __init__(self, showSampledPoint=True):
        self.invalid_samples_connections = 0
        self.invalid_samples_obstacles = 0
        self.valid_sample = 0
        self.sampledNodes = []
        self.showSampledPoint = showSampledPoint

    def add_invalid(self, obs):
        if obs:
            self.invalid_samples_obstacles += 1
        else:
            self.invalid_samples_connections += 1

    def add_free(self):
        self.valid_sample += 1

    def add_sampled_node(self, pos):
        # if pygame is not enabled, skip showing sampled point
        if not self.showSampledPoint:
            return
        self.sampledNodes.append(pos)


def check_pygame_enabled(func):
    """Pythonic decorator that force function to do Nothing
    if pygame is not enabled"""

    def wrapper(*args, **kwargs):
        if not args[0].args.enable_pygame:
            return
        return func(*args, **kwargs)

    return wrapper
