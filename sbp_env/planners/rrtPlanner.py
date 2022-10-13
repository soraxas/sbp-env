"""Represent a planner."""
import operator
from typing import List

import numpy as np

from ..planners.basePlanner import Planner
from ..utils import planner_registry
from ..utils.common import Node, Tree, Colour, Stats, PlanningOptions


class RRTPlanner(Planner):
    r"""The Rapidly-exploring random tree.

    Rapidly-exploring Random Tree* (RRT*) is an anytime, asymptotic optimal sampling-based
    motion planner that continuously updates its solution within the given budget.
    The planner itself is implemented in :class:`~planners.rrtPlanner.RRTPlanner`, while
    the default sampling
    policy is implemented in :class:`~samplers.randomPolicySampler.RandomPolicySampler`.
    """

    def __init__(self, args: PlanningOptions):
        super().__init__(args)
        self.poses = np.empty(
            (self.args.max_number_nodes * 2 + 50, self.args.engine.get_dimension())
        )  # +50 to prevent over flow
        self.c_max = float("inf")
        # this dict is to temporarily store distance of a new node to all others
        # so that in the next run other functions
        # (eg choose_least_cost_parent and rwire)
        # and take advantage to the already computed values
        self._new_node_dist_to_all_others = {}
        self.nodes = []
        # self.args.env = None  # will be set by env itself
        self.tree = Tree(self.args.engine.get_dimension())
        self.found_solution = True

        if self.args.skip_optimality:
            # respect option of skip planning for optimality
            # (i.e. reduce RRT* to RRT with the benefit of increased performance)
            def no_opt_choose_least_cost_parent(newnode, nn=None, **_):
                if nn is None:
                    raise RuntimeError("Not enough information")
                newnode.parent = nn
                newnode.cost = nn.cost + self.args.engine.dist(nn.pos, newnode.pos)
                return newnode, nn

            def no_opt_rewire(*_, **__):
                pass

            self.choose_least_cost_parent = no_opt_choose_least_cost_parent
            self.rewire = no_opt_rewire

    def init(self, **kwargs):
        """The delayed **initialisation** function

        :param start_pt: the start configuration
        :param goal_pt: the goal configuration

        :type start_pt: numpy.ndarray
        :type goal_pt: numpy.ndarray
        """
        # self.args.env = kwargs['RRT']
        self.args.sampler.init(**kwargs)
        self.start_pt = self.args.start_pt
        self.goal_pt = self.args.goal_pt

    def run_once(self):
        """Execute the main planning procedure once (mostly of the time this
        corresponds to adding one new node to the tree)

        This where the main bulk of actions happens, e.g., creating nodes or edges.
        """
        # Get an sample that is free (not in blocked space)
        rand_pos, report_success, report_fail = self.args.sampler.get_valid_next_pos()
        # Found a node that is not in X_obs
        idx = self.find_nearest_neighbour_idx(rand_pos, self.poses[: len(self.nodes)])
        nn = self.nodes[idx]
        # get an intermediate node according to step-size
        newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it has a free path to nn or not
        if not self.args.engine.cc.visible(nn.pos, newpos):
            Stats.get_instance().add_invalid(obs=False)
            report_fail(pos=rand_pos, free=False)
        else:
            newnode = Node(newpos)
            Stats.get_instance().add_free()
            self.args.sampler.add_tree_node(pos=newnode.pos)
            report_success(pos=newnode.pos, nn=nn, rand_pos=rand_pos)
            ######################
            newnode, nn = self.choose_least_cost_parent(newnode, nn, nodes=self.nodes)
            self.add_newnode(newnode)
            # rewire to see what the newly added node can do for us
            self.rewire(newnode, self.nodes)

            if self.args.engine.cc.visible(newnode.pos, self.goal_pt.pos):
                if (
                    self.args.engine.dist(newnode.pos, self.goal_pt.pos)
                    < self.args.goal_radius
                ):
                    if newnode.cost < self.c_max:
                        self.c_max = newnode.cost
                        self.goal_pt.parent = newnode
                        self.visualiser.draw_solution_path()

    def add_newnode(self, node: Node):
        """Add a new node to the tree

        :param node: node to be added

        """
        self.poses[len(self.nodes)] = node.pos
        self.nodes.append(node)

    def choose_least_cost_parent(
        self,
        newnode: Node,
        nn: Node = None,
        nodes: List[Node] = None,
        skip_optimality: bool = False,
        use_rtree: bool = False,
        poses: List[np.ndarray] = None,
    ) -> Node:
        """
        Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)

        :param newnode: the newly created node that we want to search for a new parent
        :param nn: the current closest node, optional.
        :param nodes: the list of node to search against
        :param skip_optimality: skip searching for optimality (i.e. non-asymptomatic)
        :param use_rtree: whether use rtree to store tree or not
        :param poses: list of configurations, with the same length as ``nodes``

        :return: the node with the lowest cost
        """

        if skip_optimality:
            if nn is None:
                raise RuntimeError("Not enough information")

            newnode.parent = nn
            newnode.cost = nn.cost + self.args.engine.dist(nn.pos, newnode.pos)
            return newnode, nn

        if use_rtree or poses is not None:
            nn2 = None
            if use_rtree:
                canidates = list(self.tree.nearby(newnode.pos, n=20))
            else:
                distances = np.linalg.norm(poses - newnode.pos, axis=1)
                canidates = [nodes[idx] for idx in np.argsort(distances)[:20]]

            canidates.sort(key=operator.attrgetter("cost"))

            for n in canidates:
                if self.args.engine.dist(
                    newnode.pos, n.pos
                ) <= self.args.radius and self.args.engine.cc.visible(
                    newnode.pos, n.pos
                ):
                    nn2 = n
                    break
            if nn2 is None:
                if nn is None:
                    raise RuntimeError("Unable to find nn that is connectable")
                nn2 = nn
            newnode.cost = nn2.cost + self.args.engine.dist(nn2.pos, newnode.pos)
            newnode.parent = nn2

            return newnode, nn2

        if nn is not None:
            _newnode_to_nn_cost = self.args.engine.dist(newnode.pos, nn.pos)
        self._new_node_dist_to_all_others = {}
        for p in nodes:
            _newnode_to_p_cost = self.args.engine.dist(newnode.pos, p.pos)
            self._new_node_dist_to_all_others[(newnode, p)] = _newnode_to_p_cost
            if _newnode_to_p_cost <= self.args.radius and self.args.engine.cc.visible(
                newnode.pos, p.pos
            ):
                # This is another valid parent. Check if it's better than our current one.
                if nn is None or (
                    p.cost + _newnode_to_p_cost < nn.cost + _newnode_to_nn_cost
                ):
                    nn = p
                    _newnode_to_nn_cost = _newnode_to_p_cost
        if nn is None:
            raise LookupError(
                "ERROR: Provided nn=None, and cannot find any valid nn by this function. This newnode is not close to the root tree...?"
            )
        newnode.cost = nn.cost + self.args.engine.dist(nn.pos, newnode.pos)
        assert newnode is not nn
        newnode.parent = nn

        return newnode, nn

    def rewire(
        self,
        newnode: Node,
        nodes: List[Node],
        already_rewired: bool = None,
        skip_optimality: bool = False,
        use_rtree: bool = True,
        poses: np.ndarray = None,
    ):
        """Rewire the given new node

        :param newnode: the newly created node that we want to rewires
        :param nodes: the list of node to search against
        :param already_rewired: if the node had already been rewired previously
        :param skip_optimality: skip optimality to speed up (reduce to RRT as opposed to RRT*)
        :param use_rtree: if we should use rtree as opposed to native numpy operations
        :param poses: an array of positions

        """
        if skip_optimality or len(nodes) < 1:
            return

        if use_rtree or poses is not None:
            if already_rewired is None:
                already_rewired = {newnode}
            if use_rtree:
                canidates = list(self.tree.nearby(newnode.pos, n=20))
            else:
                distances = np.linalg.norm(poses - newnode.pos, axis=1)
                canidates = [nodes[idx] for idx in np.argsort(distances)[:20]]

            canidates.sort(key=operator.attrgetter("cost"))

            for n in canidates:
                if n in already_rewired:
                    continue

                _newnode_to_n_cost = self.args.engine.dist(newnode.pos, n.pos)
                if (
                    n != newnode.parent
                    and _newnode_to_n_cost <= self.args.radius
                    and self.args.engine.cc.visible(n.pos, newnode.pos)
                    and newnode.cost + _newnode_to_n_cost < n.cost
                ):
                    # draw over the old wire
                    reconsider = n.children
                    n.parent = newnode
                    n.cost = newnode.cost + _newnode_to_n_cost
                    already_rewired.add(n)
                    self.rewire(n, reconsider, already_rewired=already_rewired)
            return

        if already_rewired is None:
            already_rewired = {newnode}
        for n in (x for x in nodes if x not in already_rewired):
            if len(already_rewired) <= 1:
                _newnode_to_n_cost = self._new_node_dist_to_all_others[newnode, n]
            else:
                _newnode_to_n_cost = self.args.engine.dist(newnode.pos, n.pos)
            if (
                n != newnode.parent
                and _newnode_to_n_cost <= self.args.radius
                and self.args.engine.cc.visible(n.pos, newnode.pos)
                and newnode.cost + _newnode_to_n_cost < n.cost
            ):
                # draw over the old wire
                reconsider = (n.parent, *n.children)
                n.parent = newnode
                n.cost = newnode.cost + _newnode_to_n_cost

    @staticmethod
    def find_nearest_neighbour_idx(pos: np.ndarray, poses: np.ndarray):
        """Find the nearest neighbour from the list of nodes

        :param pos: the position to search against the array of positions
        :param poses: the array of positions

        """
        # Make use of numpy fast parallel operation to find
        # all distance with one operation.
        distances = np.linalg.norm(poses - pos, axis=1)
        return np.argmin(distances)


# Methods for visualisation


def pygame_rrt_paint(planner: Planner) -> None:
    """Visualiser paint function for RRT

    :param planner: the planner to be visualised

    """
    planner.args.env.path_layers.fill(Colour.ALPHA_CK)
    drawn_nodes_pairs = set()
    for n in planner.nodes:
        if n.parent is not None:
            new_set = frozenset({n, n.parent})
            if new_set not in drawn_nodes_pairs:
                drawn_nodes_pairs.add(new_set)
                planner.args.env.draw_path(n, n.parent)
    if planner.goal_pt.parent is not None:
        planner.visualiser.draw_solution_path()


def klampt_draw_nodes_paint_func(planner, nodes, colour):
    drawn_nodes_pairs = set()
    for n in nodes:
        planner.args.env.draw_node(
            planner.args.engine.cc.get_eef_world_pos(n.pos), colour=colour
        )
        if n.parent is not None:
            new_set = frozenset({n, n.parent})
            if new_set not in drawn_nodes_pairs:
                drawn_nodes_pairs.add(new_set)
                planner.args.env.draw_path(
                    planner.args.engine.cc.get_eef_world_pos(n.pos),
                    planner.args.engine.cc.get_eef_world_pos(n.parent.pos),
                    colour=colour,
                )


def klampt_rrt_paint(planner: Planner) -> None:
    """Visualiser paint function for RRT

    :param planner: the planner to be visualised

    """

    colour = (1, 0, 0, 1)
    klampt_draw_nodes_paint_func(planner, planner.nodes, colour)


# start register
planner_registry.register_planner(
    "rrt",
    planner_class=RRTPlanner,
    visualise_pygame_paint=pygame_rrt_paint,
    visualise_klampt_paint=klampt_rrt_paint,
    sampler_id="random",
)
# finish register
