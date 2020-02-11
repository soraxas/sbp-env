import numpy as np
import logging

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
        self.is_start = False
        self.is_goal = False

    def __getitem__(self, x):
        return self.pos[x]

    def __len__(self):
        return len(self.pos)

    def __repr__(self):
        return f"{self.__class__.__name__}<{self.pos}>"

    def __eq__(self, other):
        return np.all(self.pos == other.pos)

    def __hash__(self):
        return hash(tuple(self.pos))

class Stats:
    def __init__(self, showSampledPoint=True):
        self.invalid_samples_connections = 0
        self.invalid_samples_obstacles = 0
        self.valid_sample = 0
        self.sampledNodes = []
        self.showSampledPoint = showSampledPoint
        self.sampler_success = 0
        self.sampler_success_all = 0
        self.sampler_fail = 0
        self.visible_cnt = 0
        self.feasible_cnt = 0

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


def update_progress(progress, total_num, num_of_blocks=10):
    if not logging.getLogger().isEnabledFor(logging.INFO):
        return
    percentage = progress / total_num
    print(
        '\r[{bar:<{num_of_blocks}}] {cur}/{total} {percen:0.1f}%'.format(
            bar='#' * int(percentage * num_of_blocks),
            cur=progress,
            total=total_num,
            percen=percentage * 100,
            num_of_blocks=num_of_blocks),
        end='')
    if percentage == 1:
        print()


class BFS:
    """Walk through the connected nodes with BFS"""

    def __init__(self, node, validNodes):
        self.visitedNodes = set()
        self.validNodes = validNodes
        self.next_node_to_visit = [node]
        self.next_node = None

    def visit_node(self, node):
        self.visitedNodes.add(node)
        self.next_node_to_visit.extend(node.edges)
        self.next_node = node

    def has_next(self):
        if self.next_node is not None:
            return True
        if len(self.next_node_to_visit) < 1:
            return False
        # get next available node
        while True:
            _node = self.next_node_to_visit.pop(0)
            if _node not in self.visitedNodes and _node in self.validNodes:
                break
            if len(self.next_node_to_visit) < 1:
                return False
        self.visit_node(_node)
        return True

    def next(self):
        node = self.next_node
        self.next_node = None
        return node


def check_pygame_enabled(func):
    """Pythonic decorator that force function to do Nothing
    if pygame is not enabled"""

    def wrapper(*args, **kwargs):
        return func(*args, **kwargs)
        if args[0].args.no_display:
            return
        return func(*args, **kwargs)

    return wrapper
