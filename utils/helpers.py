import logging
from typing import Set

import numpy as np


class MagicDict(dict):
    """Dictionary, but content is accessible like property."""

    def __deepcopy__(self, memo):
        import copy

        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

    def __getattr__(self, attr):
        """This is called what self.attr doesn't exist.

        :param attr: attribute to access
        :return: self[attr]
        """
        return self[attr]


class Colour:
    """
    Convenient class to define some RGB colour
    """

    ALPHA_CK = 255, 0, 255
    white = 255, 255, 255
    black = 20, 20, 40
    red = 255, 0, 0
    blue = 0, 0, 255
    path_blue = 26, 128, 178
    green = 0, 150, 0
    cyan = 20, 200, 200
    orange = 255, 160, 16

    @staticmethod
    def cAlpha(colour, alpha):
        """Add alpha value to the given colour

        :param colour: the base colour
        :param alpha: the desire alpha value to be added

        :return: colour with alpha
        """
        colour = list(colour)
        colour.append(alpha)
        return colour


class Node:
    """Represents a node inside a tree.

    :ivar pos: position, a.k.a., the configuration :math:`q \in C` that this node
        represents
    :ivar cost: initial value: par2
    :ivar parent: parent node
    :ivar children: children nodes
    :ivar is_start: a flag to indicate this is the start node
    :ivar is_goal: a flag to indicate this is the goal node

    :vartype pos: :class:`np.ndarray`
    :vartype cost: positive float
    :vartype parent: :class:`Node`
    :vartype children: a list of :class:`Node`
    :vartype is_start: bool
    :vartype is_goal: bool
    """

    def __init__(self, pos: np.ndarray):
        """
        :param pos: configuration of this node
        """
        self.pos: np.ndarray = np.array(pos)
        self.cost: float = 0
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
    """
    Stores statistics of a planning problem instance

    :ivar invalid_samples_connections: the number of invalid samples due to
        intermediate connections being invalid
    :ivar invalid_samples_obstacles: the number of invalid samples due to
        the sampled configurations is invalid, i.e., :math:`q \in C_\text{obs}`
    :ivar valid_sample: the number of valid samples, i.e., :math:`q \in C_\text{free}`
    :ivar sampledNodes: temporarily list of the recently sampled configurations
    :ivar showSampledPoint: A flag to denote whether we should store the list of
        recently sampled configurations
    :ivar sampler_success: UNDOCUNMENTED
    :ivar sampler_success_all: UNDOCUNMENTED
    :ivar sampler_fail: UNDOCUNMENTED
    :ivar visible_cnt: the number of calls to visibility test in the collision checker
    :ivar feasible_cnt: the number of calls to feasibility test in the collision checker


    :type invalid_samples_connections: int
    :type invalid_samples_obstacles: int
    :type valid_sample: int
    :type sampledNodes: List[:class:`Node`]
    :type showSampledPoint: bool
    :type sampler_success: int
    :type sampler_success_all: int
    :type sampler_fail: int
    :type visible_cnt: int
    :type feasible_cnt: int
    """

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
        """

        :param obs: 

        """
        if obs:
            self.invalid_samples_obstacles += 1
        else:
            self.invalid_samples_connections += 1

    def add_free(self):
        """
        Increment the free sampled point counter
        """
        self.valid_sample += 1

    def add_sampled_node(self, pos: np.ndarray):
        """Add a sampled node position

        :param pos: the position of a sampled node
        """
        # if pygame is not enabled, skip showing sampled point
        if not self.showSampledPoint:
            return
        self.sampledNodes.append(pos)

    def __repr__(self):
        return "Stats<{}>".format(
            "|".join(
                f"{attr}={getattr(self, attr)}"
                for attr in dir(self)
                if not attr.startswith("__") and not callable(getattr(self, attr))
            )
        )


def update_progress(progress: int, total_num: int, num_of_blocks: int = 10):
    """Print a progress bar

    :param progress: the current progress
    :param total_num: the total count for the progress
    :param num_of_blocks: number of blocks for the progress bar
    """
    if not logging.getLogger().isEnabledFor(logging.INFO):
        return
    percentage = progress / total_num
    print(
        "\r[{bar:<{num_of_blocks}}] {cur}/{total} {percen:0.1f}%".format(
            bar="#" * int(percentage * num_of_blocks),
            cur=progress,
            total=total_num,
            percen=percentage * 100,
            num_of_blocks=num_of_blocks,
        ),
        end="",
    )
    if percentage == 1:
        print()


class BFS:
    """Walk through the connected nodes with BFS"""

    def __init__(self, node: Node, validNodes: Set[Node]):
        """
        :param node: the starting node
        :param validNodes: the set of valid nodes that this BFS will transverse
        """
        self.visitedNodes = set()
        self.validNodes = set(validNodes)
        self.next_node_to_visit = [node]
        self.next_node = None

    def visit_node(self, node: Node):
        """Visits the given node

        :param node: the node to visit

        """
        self.visitedNodes.add(node)
        self.next_node_to_visit.extend(node.edges)
        # self.next_node_to_visit.extend(node.children)
        # try:
        #     if node.parent is not None:
        #         self.next_node_to_visit.append(node.parent)
        # except AttributeError:
        #     pass

        self.next_node = node

    def has_next(self) -> bool:
        """Check whether there's a next node for the BFS search. This function also
        performs the actual computation of finding next available node.

        :return: whether a next node is available
        """
        if self.next_node is not None:
            return True
        if len(self.next_node_to_visit) < 1:
            return False
        # get next available node
        while True:
            _node = self.next_node_to_visit.pop(0)
            if _node not in self.visitedNodes and _node in self.validNodes:
                # if _node not in self.visitedNodes:
                break
            if len(self.next_node_to_visit) < 1:
                return False
        self.visit_node(_node)
        return True

    def next(self) -> Node:
        """Get the next node

        :return: the next node from the BFS search
        """
        node = self.next_node
        self.next_node = None
        return node
