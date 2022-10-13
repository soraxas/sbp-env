import logging
from typing import List

import numpy as np
from rtree import index
import copy

from dataclasses import dataclass, asdict, fields

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sbp_env.engine import Engine
    from sbp_env.planners.basePlanner import Planner
    from sbp_env.samplers.baseSampler import Sampler
    from sbp_env.utils.planner_registry import PlannerDataPack, SamplerDataPack


@dataclass
class PlanningOptions:
    """Former: MagicDict"""

    planner_data_pack: "PlannerDataPack"
    sampler_data_pack: "SamplerDataPack"
    skip_optimality: bool
    showSampledPoint: bool
    scaling: float
    goalBias: float
    epsilon: float
    max_number_nodes: int
    radius: float
    ignore_step_size: bool
    always_refresh: bool
    rrdt_proposal_distribution: str
    start_pt: str
    goal_pt: str

    goal_radius: float = None
    no_display: bool = False
    first_solution: bool = False

    output_dir: str = None
    save_output: str = None
    rover_arm_robot_lengths: str = None

    sampler: "Sampler" = None
    planner: "Planner" = None
    engine: "Engine" = None

    def as_dict(self):
        out = {field.name: getattr(self, field.name) for field in fields(self)}
        out.update(self.extra_options)
        return out

    def __post_init__(self):
        self.extra_options = {}

    def compute_default_values(self):
        if self.radius is None:
            self.radius = self.epsilon * 1.1
        if self.goal_radius is None:
            self.goal_radius = 2 / 3 * self.radius

    def add_option(self, **kwargs):
        self.extra_options.update(kwargs)

    # def __init__(self, *args, **kwargs):
    #     super().__init__()
    #     # super().__init__(*args, **kwargs)
    #     super().__setattr__("__frozen", False)
    #
    # def __deepcopy__(self, memo):
    #     cls = self.__class__
    #     result = cls.__new__(cls)
    #     super(MagicDict, result).__setattr__(
    #         "__frozen", self.__getattribute__("__frozen")
    #     )
    #     memo[id(self)] = result
    #     for k, v in self.items():
    #         result[k] = copy.deepcopy(v, memo)
    #     return result
    # #
    # # def __getattr__(self, attr):
    # #     """This is called what self.attr doesn't exist.
    # #
    # #     :param attr: attribute to access
    # #     :return: self[attr]
    # #     """
    # #     return self[attr]
    #
    # def __setattr__(self, name, value):
    #     """This is called `m_dict.attr = XX` is called
    #
    #     :param name: the key of the attribute
    #     :param value: the value of the attribute
    #     :return: self[attr]
    #     """
    #     #     self[name] = value
    #     #
    #     # def __setitem__(self, key, value):
    #     if super().__getattribute__("__frozen") and not hasattr(self, name):
    #         raise ValueError(
    #             f"{self.__class__.__name__} is frozen but attempting to add new name "
    #             f"'{name}' to the dictionary."
    #         )
    #     super().__setattr__(name, value)
    #
    # def freeze(self):
    #     super().__setattr__("__frozen", True)
    #
    # def unfreeze(self):
    #     super().__setattr__("__frozen", False)

    def update(self, dictionary: dict):
        for k, v in dictionary.items():
            setattr(self, k, v)


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
    r"""Represents a node inside a tree.

    :ivar pos: position, a.k.a., the configuration :math:`q \in C` that this node
        represents
    :ivar cost: a positive float that represents the cost of this node
    :ivar parent: parent node
    :ivar children: children nodes
    :ivar is_start: a flag to indicate this is the start node
    :ivar is_goal: a flag to indicate this is the goal node

    :vartype pos: :class:`numpy.ndarray`
    :vartype cost: float
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
        return isinstance(other, self.__class__) and np.all(self.pos == other.pos)

    def __hash__(self):
        return hash(tuple(self.pos))


class Stats:
    r"""
    Stores statistics of a planning problem instance

    :ivar invalid_samples_connections: the number of invalid samples due to
        intermediate connections being invalid
    :ivar invalid_samples_obstacles: the number of invalid samples due to
        the sampled configurations is invalid, i.e., :math:`q \in C_\text{obs}`
    :ivar valid_sample: the number of valid samples, i.e., :math:`q \in C_\text{free}`
    :ivar sampledNodes: temporarily list of the recently sampled configurations
    :ivar showSampledPoint: A flag to denote whether we should store the list of
        recently sampled configurations
    :ivar sampler_success: UNDOCUMENTED
    :ivar sampler_success_all: UNDOCUMENTED
    :ivar sampler_fail: UNDOCUMENTED
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
    __global_stats_instance = None

    @classmethod
    def has_instance(cls) -> bool:
        return cls.__global_stats_instance is not None

    @classmethod
    def build_instance(cls, **kwargs) -> "Stats":
        if cls.__global_stats_instance is not None:
            raise ValueError(
                f"There are already an existing instance of Stats! "
                f"{cls.get_instance()}"
            )
        cls.__global_stats_instance = Stats(**kwargs)
        return cls.__global_stats_instance

    @classmethod
    def get_instance(cls) -> "Stats":
        if cls.__global_stats_instance is None:
            raise ValueError("Stats instance had not been built yet!")
        return cls.__global_stats_instance

    @classmethod
    def clear_instance(cls) -> None:
        cls.__global_stats_instance = None

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
        self.lsampler_restart_counter = 0
        self.lsampler_randomwalk_counter = 0

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

    def __init__(self, node, validNodes):
        """
        :param node: the starting node
        :param validNodes: the set of valid nodes that this BFS will transverse
        """
        self.visitedNodes = set()
        self.validNodes = set(validNodes)
        self.next_node_to_visit = [node]
        self.next_node = None

    def visit_node(self, node):
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

    def next(self):
        """Get the next node

        :return: the next node from the BFS search
        """
        if self.next_node is None and not self.has_next():
            raise StopIteration("No more node to visit")
        node = self.next_node
        self.next_node = None
        return node


class Tree:
    """
    A tree representation that stores nodes and edges.
    """

    def __init__(self, dimension: int):
        """
        :param dimension: A positive integer that represents :math:`d`,
            the dimensionality of the C-space.
        """
        p = index.Property()
        p.dimension = dimension
        self.V = index.Index(interleaved=True, properties=p)
        self.E = {}  # edges in form E[child] = parent

    def add_vertex(self, v: Node, pos: np.ndarray) -> None:
        """Add a new vertex to this tree

        :param v: Node to be added
        :param pos: The configuration :math:`q` that corresponds to the node ``v``
        """
        if len(pos) == 2:
            # print(v)
            # print(pos)
            # print(np.tile(pos, 2))
            self.V.insert(0, tuple(pos), v)
        else:
            self.V.insert(0, np.tile(pos, 2), v)
        # self.V_raw.append(v)

    def add_edge(self, child: Node, parent: Node) -> None:
        """Add a new edge to this tree

        :param child: The child node of the edge
        :param parent: The parent node of the edge

        """
        self.E[child] = parent

    def nearby(self, x: np.ndarray, n: int) -> List[Node]:
        """Find ``n`` many nearby nodes that are closest to a given position

        :param x: Position
        :param n: Max number of results

        """
        return self.V.nearest(np.tile(x, 2), num_results=n, objects="raw")

    def get_nearest(self, x: np.ndarray) -> Node:
        """Get the closest node

        :param x: Position

        :return: the closest node
        """
        return next(self.nearby(x, 1))

    # def connect_to_point(self, tree, x_a, x_b):
    #     """
    #     Connect vertex x_a in tree to vertex x_b
    #     :param tree: int, tree to which to add edge
    #     :param x_a: tuple, vertex
    #     :param x_b: tuple, vertex
    #     :return: bool, True if able to add edge, False if prohibited by an obstacle
    #     """
    #     if self.V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
    #         self.add_vertex(tree, x_b)
    #         self.add_edge(tree, x_b, x_a)
    #         return True
    #     return False

    # def can_connect_to_goal(self, tree):
    #     """
    #     Check if the goal can be connected to the graph
    #     :param tree: rtree of all Vertices
    #     :return: True if can be added, False otherwise
    #     """
    #     x_nearest = self.get_nearest(tree, self.x_goal)
    #     if self.x_goal in self.E and x_nearest in self.E[self.x_goal]:
    #         # tree is already connected to goal using nearest vertex
    #         return True
    #     if self.X.collision_free(x_nearest, self.x_goal, self.r):  # check if obstacle-free
    #         return True
    #     return False


try:
    from functools import cached_property

except ImportError:
    import functools

    class cached_property:
        """Decorator that caches a function's return value each time it is called.
        If called later with the same arguments, the cached value is returned, and
        not re-evaluated.
        """

        def __init__(self, func):
            self.func = func
            self.cache = {}

        def __call__(self, *args):
            try:
                return self.cache[args]
            except KeyError:
                value = self.func(*args)
                self.cache[args] = value
                return value
            except TypeError:
                # uncachable -- for instance, passing a list as an argument.
                # Better to not cache than to blow up entirely.
                return self.func(*args)

        def __repr__(self):
            """Return the function's docstring."""
            return self.func.__doc__

        def __get__(self, obj, objtype):
            """Support instance methods."""
            return functools.partial(self.__call__, obj)()
