import numpy as np
from overrides import overrides

from ..env import Node
from ..planners.rrtPlanner import RRTPlanner
from ..utils import planner_registry
from ..utils.common import Colour, Stats


# noinspection PyAttributeOutsideInit
class BiRRTPlanner(RRTPlanner):
    r"""The bidrectional RRT* planner, or sometimes it's also referred to as the
    *RRT-Connect\**.

    The class :class:`~planners.birrtPlanner.BiRRTPlanner` uses an adopted version of
    random policy sampler that makes it suitable for using in both the start and goal
    trees, which is implemented in :class:`~samplers.birrtSampler.BiRRTSampler`.
    """

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.goal_tree_nodes = []
        self.goal_tree_poses = np.empty(
            (
                self.args.max_number_nodes * 2 + 50,  # +50 to prevent over flow
                self.args.engine.get_dimension(),
            )
        )
        self.goal_tree_nodes.append(self.args.env.goal_pt)
        self.goal_tree_poses[0] = self.args.env.goal_pt.pos

        self.found_solution = False
        self.goal_tree_turn = False

    @overrides
    def run_once(self):
        if self.goal_tree_turn and not self.found_solution:
            # extend from goal tree
            poses = self.goal_tree_poses
            nodes = self.goal_tree_nodes
        else:
            # extend from init tree
            poses = self.poses
            nodes = self.nodes
        self.goal_tree_turn = not self.goal_tree_turn
        # check two tree join together

        ###################################################################
        # Get an sample that is free (not in blocked space)
        rand_pos, report_success, report_fail = self.args.sampler.get_valid_next_pos()
        # Found a node that is not in X_obs

        idx = self.find_nearest_neighbour_idx(rand_pos, poses[: len(nodes)])
        nn = nodes[idx]
        # get an intermediate node according to step-size
        newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it has a free path to nn or not
        if not self.args.engine.cc.visible(nn.pos, newpos):
            Stats.get_instance().add_invalid(obs=False)
            report_fail(pos=rand_pos, free=False)
        else:
            newnode = Node(newpos)
            Stats.get_instance().add_free()
            report_success(pos=newnode.pos, nn=nn, rand_pos=rand_pos)
            newnode, nn = self.choose_least_cost_parent(newnode, nn, nodes=nodes)
            poses[len(nodes)] = newnode.pos

            nodes.append(newnode)
            # rewire to see what the newly added node can do for us
            self.rewire(newnode, nodes)

            ###################################################################
            # check if two tree joins
            if not self.found_solution:
                if nodes is self.nodes:
                    other_poses = self.goal_tree_poses
                    other_nodes = self.goal_tree_nodes
                else:
                    other_poses = self.poses
                    other_nodes = self.nodes
                distances = np.linalg.norm(
                    other_poses[: len(self.nodes)] - newpos, axis=1
                )
                if min(distances) < self.args.epsilon:
                    idx = np.argmin(distances)
                    if self.args.engine.cc.visible(other_poses[idx], newpos):

                        self.found_solution = True
                        # get the two closest nodes
                        if nodes is self.nodes:
                            init_tree_node = newnode
                            goal_tree_node = other_nodes[idx]
                        else:
                            init_tree_node = other_nodes[idx]
                            goal_tree_node = newnode
                        _nextnode = goal_tree_node  # keep track of original parent
                        _old_parent = _nextnode.parent

                        # trees joined! Flip all the parent as child
                        nn = init_tree_node

                        assert init_tree_node in self.nodes
                        assert goal_tree_node in self.goal_tree_nodes

                        to_be_removed = []
                        while _old_parent is not None:
                            _old_parent = _nextnode.parent

                            _nextnode, nn = self.choose_least_cost_parent(
                                _nextnode, nn=nn, nodes=self.nodes
                            )
                            self.rewire(_nextnode, nodes=self.nodes)

                            self.poses[len(self.nodes)] = _nextnode.pos
                            self.nodes.append(_nextnode)
                            to_be_removed.append(_nextnode)

                            nn = _nextnode
                            _nextnode = _old_parent

            if self.goal_pt.parent is not None:
                if self.goal_pt.parent.cost < self.c_max:
                    self.c_max = self.goal_pt.parent.cost


def pygame_birrt_planner_paint(planner):
    """Visualisation function for BiRRT

    :param planner: planner to be visualised

    """
    planner.args.env.path_layers.fill(Colour.ALPHA_CK)
    drawn_nodes_pairs = set()
    for nodes in (planner.nodes, planner.goal_tree_nodes):
        for n in nodes:
            if n.parent is not None:
                new_set = frozenset({n, n.parent})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)
                    planner.args.env.draw_path(n, n.parent)
    if planner.goal_pt.parent is not None:
        planner.visualiser.draw_solution_path()


from ..planners.rrtPlanner import klampt_draw_nodes_paint_func


def klampt_birrt_paint(planner):
    """Visualiser paint function for BiRRT

    :param planner: the planner to be visualised

    """
    for c, nodes in (
        ((1, 0, 0, 1), planner.nodes),
        ((0, 0, 1, 1), planner.goal_tree_nodes),
    ):
        klampt_draw_nodes_paint_func(planner, nodes, c)


# start register
planner_registry.register_planner(
    "birrt",
    planner_class=BiRRTPlanner,
    visualise_pygame_paint=pygame_birrt_planner_paint,
    visualise_klampt_paint=klampt_birrt_paint,
    sampler_id="birrt_sampler",
)
# finish register
