from __future__ import annotations

import logging
import math
import random
from typing import Optional, Tuple, List

import numpy as np
from overrides import overrides
from tqdm import tqdm

from ..planners.rrtPlanner import RRTPlanner
from ..samplers.baseSampler import Sampler
from ..samplers.randomPolicySampler import RandomPolicySampler
from ..utils import planner_registry
from ..utils.common import BFS, PlanningOptions, Colour, Stats

LOGGER = logging.getLogger(__name__)

MAX_NUMBER_NODES = 20000

RANDOM_RESTART_EVERY = 20
ENERGY_START = 10
RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 0.1  # .75


def kernel(
    x: np.ndarray,
    xprime: np.ndarray,
    sigma: float = 0.1,
    length_scale: float = np.pi / 4,
):
    """Kernel function to be used for Bayesian RRdT

    :param x: bins
    :param xprime: origin of x
    :param sigma: sigma for the kernel
    :param length_scale: lambda for the kernel

    """
    return sigma**2 * np.exp(
        -(2 * np.sin((np.linalg.norm(x - xprime[:, None], axis=0)) / 2) ** 2)
        / (length_scale**2)
    )


class DisjointTreeParticle:
    """RRdT's sampler particle"""

    def __init__(
        self,
        proposal_type: str,
        planner: RRdTPlanner,
        p_manager: MABScheduler,
        particle_idx: int,
        pos: np.ndarray,
        isroot: Optional[DTreeType] = None,
    ):
        self.p_manager = p_manager
        self.planner = planner
        self.tree = None
        self.idx = particle_idx

        if isroot is not None:
            # root particle's tree is given, as oppose to be spawned by the particle
            self._reset(pos)
            self.tree = isroot
        else:
            self.restart(pos=pos, restart_when_merge=False)

        ##############################

        self.last_failed = False
        self.kappa = np.pi * 1.5
        self.succeed = 0
        self.failed = 0
        self.failed_reset = 0
        self.last_node = None

        if proposal_type not in ("dynamic-vonmises", "ray-casting", "original"):
            raise Exception("Given proposal type is not supported.")
        self.proposal_type = proposal_type
        self.last_origin = None
        self.A = None

    def register_tree(self, tree: DTreeType):
        tree.particle_handlers.append(self)

    def deregister_tree(self, tree: DTreeType):
        tree.particle_handlers.remove(self)

    def _reset(self, pos: Optional[np.ndarray] = None):
        """Restarts this local sampler at somewhere else.

        :param pos:  (Default value = None)

        """
        if pos is None:
            # I cant really get started...can i?
            raise Exception("No pos given")
        # self.energy = 1
        self.pos = np.copy(pos)

        self._trying_this_pos = np.copy(pos)
        self.provision_dir = None
        self.succeed = 0
        self.failed = 0
        self.failed_reset = 0

    def restart(
        self,
        pos: Optional[np.ndarray] = None,
        restart_when_merge: bool = True,
    ):
        merged_tree = None
        if pos is None:
            # get random position
            pos = self.p_manager.new_pos_in_free_space()
            merged_tree = self.planner.add_pos_to_existing_tree(Node(pos), None)
            if merged_tree is not None and restart_when_merge:
                # Successfully found a new valid node that's close to existing tree
                # Return False to indicate it
                # (and abort restart if we want more exploration)
                self.p_manager.add_to_restart(self)
                # we need to abort the restart procedure. add this to pending restart
                return False
        if self.tree is not None:
            self.deregister_tree(self.tree)
        # initialise to initial value, create new d-tree
        if merged_tree is not None:
            self.tree = merged_tree
            merged_tree.particle_handlers.append(self)
        else:
            # spawn a new tree
            self.tree = TreeDisjoint(dim=self.p_manager.args.engine.get_dimension())
            self.register_tree(self.tree)
            self.tree.add_newnode(Node(pos))
            self.planner.add_tree(self.tree)
        self.p_manager.modify_energy(idx=self.idx, set_val=ENERGY_START)
        self._reset(pos)
        # this particle is ready to sample again. Remove it from the pending list,
        # if exists
        try:
            self.p_manager.local_samplers_to_be_rstart.remove(self)
        except ValueError:
            pass
        return True

    def confirm(self, pos: np.ndarray):
        self.pos = pos
        self.dir = self.provision_dir

    @staticmethod
    def rand_unit_vecs(num_dims: int, number: int):
        vec = np.random.standard_normal((number, num_dims))
        vec = vec / np.linalg.norm(vec, axis=1)[:, None]
        return vec

    @staticmethod
    def generate_pmf(
        num_dims: int,
        mu: np.ndarray,
        kappa: float,
        unit_vector_support: Optional[np.ndarray] = None,
        num: int = None,
        plot: bool = False,
    ):
        assert num_dims >= 1
        if num is None:
            num = 361 * (num_dims - 1) ** 2
        ####
        if unit_vector_support is None:
            unit_vector_support = DisjointTreeParticle.rand_unit_vecs(num_dims, num).T
        pmf = np.exp(kappa * mu.dot(unit_vector_support))
        pmf = pmf / pmf.sum()
        return unit_vector_support, pmf

    def draw_sample(self, origin=None) -> np.ndarray:
        """Return sample points from this particle

        :param origin:  (Default value = None)

        """
        if self.proposal_type in ("dynamic-vonmises", "original"):

            if self.last_origin is None:
                # first time
                mu = np.random.standard_normal(
                    (1, self.p_manager.args.engine.get_dimension())
                )
                mu = mu / np.linalg.norm(mu, axis=1)[:, None]
                self.provision_dir = mu[0]
                return self.provision_dir
            elif self.A is None:
                self.x, self.y = self.generate_pmf(
                    num_dims=self.p_manager.args.engine.get_dimension(),
                    mu=self.last_origin,
                    kappa=2,
                )

                self.A = self.y.copy()
                # self.last_origin = mu

        if origin is None:
            # use self direction if none is given.
            origin = self.dir

        # self.last_origin = origin

        # use argmax or draw probabilistically
        if self.proposal_type == "ray-casting":
            if not self.last_failed:
                # skip drawing if we haven't failed (if we are using ray-casting)
                # this should return the origin of where we came from
                return origin
            x_idx = y_idx = np.argmax(self.A)
            xi = self.x[x_idx]

        elif self.proposal_type == "dynamic-vonmises":
            # bin_width = self.x[1] - self.x[0]
            xi_idx = np.random.choice(range(self.x.shape[1]), p=self.A)
            xi = self.x[:, xi_idx]
            # xi = np.random.uniform(xi, xi + bin_width)

        elif self.proposal_type == "original":
            # bin_width = self.x[1] - self.x[0]
            xi_idx = np.random.choice(range(self.x.shape[1]), p=self.A)
            xi = self.x[:, xi_idx]
            # xi = np.random.uniform(xi, xi + bin_width)

        else:
            raise ValueError("Unsupported proposal type")
        return xi  # + origin


class RRdTSampler(Sampler):
    """Represents RRdT's sampler"""

    def __init__(self, restart_when_merge: bool = True, num_dtrees: int = 4, **kwargs):
        super().__init__(**kwargs)
        self.restart_when_merge = restart_when_merge
        self.num_dtrees = num_dtrees
        self._last_prob = None

        self._c_random = 0
        self.last_choice = 0
        self.last_failed = True

    def _add_particle(
        self, pos: np.ndarray, isroot: Optional[DTreeType] = None
    ) -> DisjointTreeParticle:
        self.p_manager.particles.append(
            DisjointTreeParticle(
                proposal_type=self.args.rrdt_proposal_distribution,
                planner=self.args.planner,
                pos=pos,
                p_manager=self.p_manager,
                particle_idx=len(self.p_manager.particles),
                isroot=isroot,
            )
        )
        # return the reference to the newly created particle
        return self.p_manager.particles[-1]

    def init(self, **kwargs):
        super().init(**kwargs)
        # For benchmark stats tracking
        Stats.get_instance().lsampler_restart_counter = 0
        Stats.get_instance().lsampler_randomwalk_counter = 0

        self.random_sampler = RandomPolicySampler()
        self.random_sampler.init(**kwargs)

        self.p_manager = MABScheduler(
            num_dtrees=self.num_dtrees,
            start_pt=self.start_pos,
            goal_pt=self.goal_pos,
            args=self.args,
            random_sampler=self.random_sampler,
        )

        global MAX_NUMBER_NODES
        MAX_NUMBER_NODES = self.args.max_number_nodes

        assert self.p_manager.num_dtrees >= 2
        for _ in range(
            self.p_manager.num_dtrees - 2
        ):  # minus two for start and goal point
            self._add_particle(pos=self.p_manager.new_pos_in_free_space())
        # spawn one that comes from the goal
        goal_dt_p = self._add_particle(pos=self.goal_pos)
        goal_dt_p.tree.add_newnode(self.args.env.goal_pt)

        # spawn one that comes from the root
        self.args.planner.root = TreeRoot(dim=self.args.engine.get_dimension())
        root_particle = self._add_particle(
            pos=self.start_pos, isroot=self.args.planner.root
        )
        root_particle.register_tree(self.args.planner.root)
        root_particle.tree.add_newnode(self.args.env.start_pt)

    def particles_random_free_space_restart(self):
        r"""Randomly restarts particle in :math:`C_\text{free}`"""
        for i in range(self.p_manager.num_dtrees):
            if self.p_manager.dtrees_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                self.p_manager.add_to_restart(self.p_manager.particles[i])

    def report_success(self, idx: int, **kwargs):
        """Report that the sample returned by particle with index ``idx`` was
        successful

        :param idx: the index of the particle
        :param newnode: the node that was created

        """
        self.p_manager.particles[idx].last_node = kwargs["newnode"]
        self.p_manager.confirm(idx, kwargs["pos"])
        self.last_failed = False

        particle = self.p_manager.particles[idx]
        # update pmf
        particle.succeed += 1
        particle.failed_reset = 0
        particle.last_origin = particle.provision_dir
        if particle.proposal_type in ("dynamic-vonmises", "ray-casting", "original"):
            # reset to the original von mises
            # TODO make a sharper von mises distribution (higher kappa) when success
            # particle.A = particle.y.copy()
            particle.A = None

            particle.last_failed = False

    def report_fail(self, idx: int, **kwargs):
        """Reports that the sampled position from the particle had failed

        :param idx: the index of the particle

        """
        self.last_failed = True
        if idx >= 0:
            particle = self.p_manager.particles[idx]
            self.p_manager.modify_energy(idx=idx, factor=0.7)
            # update pmf
            particle.failed_reset += 1
            particle.failed += 1
            if particle.proposal_type in ("dynamic-vonmises", "ray-casting"):
                if particle.last_origin is None:
                    # still in phrase 1
                    return
                particle.last_failed = True

                # get previous trying direction
                xi = particle.provision_dir

                # revert effect of shifting origin
                xi -= particle.last_origin

                # find cloest x idx
                particle.A = particle.A - kernel(
                    particle.x,
                    xi,
                    sigma=np.sqrt(particle.A) * 0.9,
                    length_scale=np.pi / 10,
                )
                particle.A = particle.A / np.linalg.norm(particle.A, ord=1)

    def restart_all_pending_local_samplers(self):
        """Restarts all disjointed-tree particle that are pending to be restarts"""
        # restart all pending local samplers
        if len(self.p_manager.local_samplers_to_be_rstart) > 0:
            # during the proces of restart, if the new restart position
            # is close to an existing tree, it will simply add to that new tree.
            if not self.p_manager.local_samplers_to_be_rstart[0].restart(
                restart_when_merge=self.restart_when_merge
            ):
                # This flag denotes that a new position was found among the trees,
                # And it NEEDS to get back to restarting particles in the next ierations
                return False
            return False
        return True

    def get_next_pos(self):
        self._c_random += 1

        if self._c_random > RANDOM_RESTART_EVERY > 0:
            self._c_random = 0
            self.particles_random_free_space_restart()
        if not self.restart_all_pending_local_samplers():
            LOGGER.debug("Adding node to existing trees.")
            return None
        # get a node to random walk
        choice = self.get_random_choice()

        pos = self.random_walk(choice)
        # pos, choice = self.random_walk_by_mouse()
        return (
            pos,
            self.p_manager.particles[choice].tree,
            self.p_manager.particles[choice].last_node,
            lambda c=choice, **kwargs: self.report_success(c, **kwargs),
            lambda c=choice, **kwargs: self.report_fail(c, **kwargs),
        )

    def random_walk_by_mouse(self):
        """Random walk by mouse

        .. warning::
            For testing purpose. Mimic random walk, but do so via mouse click.
        """
        from samplers.mouseSampler import MouseSampler as mouse

        pos = mouse.get_mouse_click_position(scaling=self.args.scaling)
        # find the cloest particle from this position
        _dist = None
        p_idx = None
        for i in range(len(self.p_manager.particles)):
            p = self.p_manager.particles[i]
            if _dist is None or _dist > self.args.engine.dist(pos, p.pos):
                _dist = self.args.engine.dist(pos, p.pos)
                p_idx = i
        LOGGER.debug("num of tree: {}".format(len(self.args.planner._disjointed_trees)))
        self.p_manager.new_pos(idx=p_idx, pos=pos, dir=0)
        return pos, p_idx

    def random_walk(self, idx: int):
        """Performs a random walk for the particle at the given index

        :param idx: the index of the particle

        """
        Stats.get_instance().lsampler_randomwalk_counter += 1
        # Randomly bias toward goal direction
        if False and random.random() < self.args.goalBias:
            dx = self.goal_pos[0] - self.p_manager.get_pos(idx)[0]
            dy = self.goal_pos[1] - self.p_manager.get_pos(idx)[1]
            goal_direction = math.atan2(dy, dx)
            new_direction = self.p_manager.particles[idx].draw_sample(
                origin=goal_direction
            )
        else:
            new_direction = self.p_manager.particles[idx].draw_sample()

        new_pos = self.p_manager.get_pos(idx) + new_direction * self.args.epsilon * 3
        self.p_manager.new_pos(idx=idx, pos=new_pos, dir=new_direction)
        return new_pos

    def get_random_choice(self):
        """Get a random particle (disjointed tree) from the currently managed particiles

        :return: Node from p_manager
        """
        if self.p_manager.num_dtrees == 1:
            return 0

        prob = self.p_manager.get_prob()
        self._last_prob = prob  # this will be used to paint particles
        try:
            choice = np.random.choice(range(self.p_manager.num_dtrees), p=prob)
            assert self.p_manager.particles[choice].tree is not None
            assert hasattr(self.p_manager.particles[choice].tree, "poses"), hex(
                id(self.p_manager.particles[choice].tree)
            )
        except ValueError as e:
            # NOTE dont know why the probability got out of sync... (not sums to 1)
            # probably because of underflow?
            # We will notify the use, then try re-sync the prob
            LOGGER.error(
                "!! probability got exception '{}'... trying to re-sync prob again.".format(
                    e
                )
            )
            self.p_manager.resync_prob()
            prob = self.p_manager.get_prob()
            self._last_prob = prob
            choice = np.random.choice(range(self.p_manager.num_dtrees), p=prob)
        self.last_choice = choice
        return choice


############################################################
##    PATCHING RRT with disjointed-tree specific stuff    ##
############################################################


class Node:
    """Overloads the Tree-based node with extra info"""

    def __init__(self, pos: np.ndarray):
        self.pos = np.array(pos)
        self.cost = 0  # index 0 is x, index 1 is y
        self.edges = []
        self.children = []
        self.is_start = False
        self.is_goal = False

    def __repr__(self):
        try:
            num_edges = len(self.edges)
        except AttributeError:
            num_edges = "DELETED"
        return "Node(pos={}, cost={}, num_edges={})".format(
            self.pos, self.cost, num_edges
        )


class RRdTPlanner(RRTPlanner):
    r"""The Rapidly-exploring Random disjointed-Trees.
    The RRdT* planner is implemented based on Lai *et. al.*'s [#Lai]_ work.
    The main idea is that the planner keeps a pool of disjointed trees

    .. math::
        \mathbb{T}=\{\mathcal{T}_\text{root}, \mathcal{T}_1, \ldots, \mathcal{T}_k\}

    where it consists of a rooted tree that connects to the :math:`q_\text{start}`
    starting configuration, and :math:`k` many disjointed trees that randomly explores
    *C-Space*.
    Each disjointed tree is modelled as an arm in the Multi-Armed Bandit problem,
    i.e. each :math:`\mathcal{T}_i` has an arm :math:`a_i`, where the probability to
    draw each arm is dependent on its previous success as given by

    .. math::
        \mathbb{P}(a_{i,t} \mid a_{i,t-1}, o_{t-1})\,\forall_{i\in\{1,...,k\}}

    with :math:`o_{t-1}` as the arm :math:`a_i`'s previous observation.


    .. [#Lai] Lai, Tin, Fabio Ramos, and Gilad Francis. "Balancing global
        exploration and local-connectivity exploitation with rapidly-exploring random
        disjointed-trees." 2019 International Conference on Robotics and Automation (
        ICRA). IEEE, 2019.

    """

    def __init__(self, args: PlanningOptions):
        super().__init__(args)
        self.root = None
        self._disjointed_trees = []

    @overrides
    def run_once(self):
        # Get an sample that is free (not in blocked space)
        while True:
            _tmp = self.args.sampler.get_next_pos()
            if _tmp is None:
                # This denotes a particle had tried to restart and added the new node
                # to existing tree instead.
                # Skip remaining steps and iterate to next loop
                break
            rand_pos = _tmp[0]
            Stats.get_instance().add_sampled_node(rand_pos)
            if self.args.engine.cc.feasible(rand_pos):
                Stats.get_instance().sampler_success += 1
                break
            report_fail = _tmp[-1]
            report_fail(pos=rand_pos, obstacle=True)
            Stats.get_instance().add_invalid(obs=True)
            Stats.get_instance().sampler_fail += 1

        Stats.get_instance().sampler_success_all += 1

        if _tmp is None:
            # we have added a new samples when respawning a local sampler
            return
        rand_pos, parent_tree, last_node, report_success, report_fail = _tmp
        if last_node is not None and False:
            # use the last succesful node as the nearest node
            # This is expliting the advantage of local sampler :)
            nn = last_node
            newpos = rand_pos
        else:
            idx = self.find_nearest_neighbour_idx(
                rand_pos, parent_tree.poses[: len(parent_tree.nodes)]
            )
            nn = parent_tree.nodes[idx]
            # get an intermediate node according to step-size
            newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it is free or not
        if not self.args.engine.cc.visible(nn.pos, newpos):
            Stats.get_instance().add_invalid(obs=False)
            report_fail(pos=rand_pos, free=False)
        else:
            newnode = Node(newpos)

            Stats.get_instance().add_free()
            self.args.sampler.add_tree_node(pos=newnode.pos)
            report_success(newnode=newnode, pos=newnode.pos)
            ######################
            newnode, nn = self.connect_two_nodes(newnode, nn, parent_tree)
            # try to add this newnode to existing trees
            self.add_pos_to_existing_tree(newnode, parent_tree)

    def rrt_star_add_node(self, newnode: Node, nn: Optional[Node] = None):
        """This function perform finding optimal parent, and rewiring.

        :param newnode: the node to add to the tree
        :param nn: an approximate of nearest node

        """

        newnode, nn = self.choose_least_cost_parent(
            newnode, nn=nn, nodes=self.root.nodes
        )
        self.rewire(newnode, nodes=self.root.nodes)

        # newnode.parent = nn

        # check for goal condition
        if self.args.engine.dist(newnode.pos, self.goal_pt.pos) < self.args.goal_radius:
            if self.args.engine.cc.visible(newnode.pos, self.goal_pt.pos):
                if newnode.cost < self.c_max:
                    self.c_max = newnode.cost
                    self.goal_pt.parent = newnode
                    newnode.children.append(self.goal_pt.parent)
        # add node to tree
        self.add_newnode(newnode)
        return newnode, nn

    ##################################################
    # Tree management:
    ##################################################
    def connect_two_nodes(
        self, newnode: Node, nn: Optional[Node], parent_tree: Optional[DTreeType] = None
    ):
        """Add node to disjoint tree OR root tree.

        :param newnode: the new node to connects
        :param nn: a node from the existing tree to be connected
        :param parent_tree: if given, add newnode to this tree

        """

        if parent_tree is self.root:
            # using rrt* algorithm to add each nodes
            newnode, nn = self.rrt_star_add_node(newnode, nn)
        else:
            newnode.edges.append(nn)
            nn.edges.append(newnode)
        if parent_tree is not None:
            parent_tree.add_newnode(newnode)
        return newnode, nn

    def add_pos_to_existing_tree(
        self, newnode: Node, parent_tree: Optional[DTreeType]
    ) -> Optional[DTreeType]:
        """Try to add pos to existing tree. If success, return True.

        :param newnode: the node to be added
        :param parent_tree: the tree to add the node

        """
        nearest_nodes = self.find_nearest_node_from_neighbour(
            node=newnode, parent_tree=parent_tree, radius=self.args.epsilon
        )
        cnt = 0
        for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
            # for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
            if self.args.engine.cc.visible(newnode.pos, nearest_neighbour_node.pos):
                if parent_tree is None:
                    ### joining ORPHAN NODE to a tree
                    self.connect_two_nodes(
                        newnode, nearest_neighbour_node, nearest_neighbour_tree
                    )
                    parent_tree = nearest_neighbour_tree
                else:
                    ### joining a TREE to another tree
                    # try:
                    parent_tree = self.join_trees(
                        parent_tree,
                        nearest_neighbour_tree,
                        tree1_node=newnode,
                        tree2_node=nearest_neighbour_node,
                    )
                    # return parent_tree
                    # except AssertionError as e:
                    #     raise e
                    #     LOGGER.warning(
                    #         "Assertion error in joining sampled pt to existing tree."
                    #         "Skipping this node..."
                    #     )
                cnt += 1
                if cnt > 5:
                    break
        return parent_tree

    def find_nearest_node_from_neighbour(
        self, node: Node, parent_tree: DTreeType, radius: float
    ) -> List[Tuple[Node, DTreeType]]:
        """Given a tree, a node within that tree, and radius
        Return a list of cloest nodes (and its corresponding tree) within the radius (that's from other neighbourhood trees)

        :param node: the node to be added
        :param parent_tree: the tree to add the given node
        :param radius: the maximum radius to add the given node

        :returns: a list of potential nodes

        """

        # IF root exists in the list, add it at the last position (So the connection
        # behaviour would remain stable)
        # This ensure all previous action would only add add edges to each nodes,
        # and only the last action would it modifies the entire tree structures with
        # rrt* procedures.
        nearest_nodes = {}
        for tree in [*self._disjointed_trees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            idx = self.find_nearest_neighbour_idx(
                node.pos, tree.poses[: len(tree.nodes)]
            )
            nn = tree.nodes[idx]
            if self.args.engine.dist(nn.pos, node.pos) < radius:
                nearest_nodes[tree] = nn
        # construct list of the found solution.
        # And root at last (or else the result won't be stable)
        root_nn = nearest_nodes.pop(self.root, None)
        nearest_nodes_list = [(nearest_nodes[key], key) for key in nearest_nodes]
        if root_nn is not None:
            nearest_nodes_list.append((root_nn, self.root))
        return nearest_nodes_list

    def join_tree_to_root(
        self, tree: DTreeType, middle_node: Node, root_tree_node: Node
    ):
        """It will join the given tree to the root

        :param tree: the disjointed tree to be added to root tree
        :param middle_node: the middle node that connects the disjointed tree and the
            root tree
        :param root_tree_node: a node from the root tree

        """
        # from env import Colour
        bfs = BFS(middle_node, validNodes=tree.nodes)
        # add all nodes from disjoint tree via rrt star method
        LOGGER.info("> Joining to root tree")
        with tqdm(desc="join to root", total=len(tree.nodes)) as pbar:

            nn = middle_node

            bfs = BFS(middle_node, validNodes=tree.nodes)
            nn = root_tree_node
            while bfs.has_next():
                newnode = bfs.next()
                pbar.update()
                try:
                    newnode, nn = self.connect_two_nodes(
                        newnode, nn=None, parent_tree=self.root
                    )
                    nn = newnode
                except LookupError:
                    LOGGER.warning(
                        "nn not found when attempting to joint to root. Ignoring..."
                    )
                # remove this node's edges (as we don't have a use on them anymore)
                # to free memory
                del newnode.edges

        # assert progress == total_num, "Inconsistency in BFS walk {} != {}".format(
        #     progress, total_num)

    def join_trees(
        self,
        tree1: DTreeType,
        tree2: DTreeType,
        tree1_node: Node,
        tree2_node: Node,
    ):
        r"""Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.

        tree1_node & 2 represent the nodes that join the two tree together.
        It only matters currently to joining root tree to disjointed tree itself.

        :param tree1: disjointed tree 1 :math:`\mathcal{T}_1`
        :param tree2: disjointed tree 2 :math:`\mathcal{T}_2`
        :param tree1_node: a node from tree 1 :math:`v_1 \in \mathcal{T}_1`
        :param tree2_node: a node from tree 2 :math:`v_2 \in \mathcal{T}_2`

        """
        assert tree1 is not tree2, "Both given tree should not be the same"

        def assert_tree_in_dtrees(tree, value):
            assert (
                tree in self._disjointed_trees
            ) == value, (
                f"Checks 'tree in self._disjointed_trees == {value}' fails, {tree}"
            )

        if tree1 is self.root:
            assert_tree_in_dtrees(tree1, False)
            assert_tree_in_dtrees(tree2, True)
        elif tree2 is self.root:
            assert_tree_in_dtrees(tree1, True)
            assert_tree_in_dtrees(tree2, False)
        else:
            assert_tree_in_dtrees(tree1, True)
            assert_tree_in_dtrees(tree2, True)

        LOGGER.info(
            " => Joining trees with size {} to {}".format(
                len(tree1.nodes), len(tree2.nodes)
            )
        )
        # Re-arrange only. Make it so that tree1 will always be root (if root exists among the two)
        # And tree1 node must always be belong to tree1, tree2 node belong to tree2
        if tree1 is not self.root:
            # set tree1 as root (if root exists among the two)
            tree1, tree2 = tree2, tree1
        if tree1_node in tree2.nodes or tree2_node in tree1.nodes:
            # swap to correct position
            tree1_node, tree2_node = tree2_node, tree1_node

        if tree1 is self.root:
            # find which middle_node belongs to the disjointed tree
            self.join_tree_to_root(tree2, tree2_node, root_tree_node=tree1_node)
            # self.connect_two_nodes(tree1_node, tree2_node, draw_only=True)
        else:
            self.connect_two_nodes(tree1_node, tree2_node)
            tree1.extend_tree(tree2)
        del tree2.nodes
        del tree2.poses

        # remove the tree from the list of trees first, so that when the particle is
        # restarting it wont try to connects back to the, now non-existence, tree.
        self.remove_tree(tree2)

        if self.args.sampler.restart_when_merge:
            # restart all particles
            for p in tree2.particle_handlers:
                self.args.sampler.p_manager.add_to_restart(p)
                p.tree = None
                # p.restart()
            del tree2.particle_handlers
        else:
            # pass the remaining particle to the remaining tree
            for p in tree2.particle_handlers:
                p.tree = tree1
                tree1.particle_handlers.append(p)

        return tree1

    # Tree management
    def add_tree(self, tree):
        self._disjointed_trees.append(tree)

    def remove_tree(self, tree):
        self._disjointed_trees.remove(tree)


############################################################
# d-tree classes
############################################################


class DTreeType:
    """Abstract d-tree type"""

    def __init__(self, dim: int):
        self.particle_handlers: List[DisjointTreeParticle] = []
        self.nodes = []
        self.poses = np.empty(
            (MAX_NUMBER_NODES * 2 + 50, dim)
        )  # +50 to prevent over flow
        # This stores the last node added to this tree (by local sampler)

    def add_newnode(self, node: Node):
        """Add a new node to the d-tree

        :param node: the node to be added

        """
        self.poses[len(self.nodes)] = node.pos
        self.nodes.append(node)

    def extend_tree(self, tree: DTreeType):
        """Extend nodes from the given tree to this tree

        :param tree: the tree to be extended

        """
        self.poses[len(self.nodes) : len(self.nodes) + len(tree.nodes)] = tree.poses[
            : len(tree.nodes)
        ]
        self.nodes.extend(tree.nodes)

    def __repr__(self):
        string = super().__repr__()
        string += "\n"
        import pprint

        string += pprint.pformat(vars(self), indent=4)
        # string += ', '.join("%s: %s" % item for item in vars(self).items())
        return string


class TreeRoot(DTreeType):
    """Represents the root"""

    pass


class TreeDisjoint(DTreeType):
    """Represents one d-tree"""

    pass


class MABScheduler:
    """Scheduler for the MAB procedure"""

    def __init__(
        self,
        num_dtrees: int,
        start_pt: np.ndarray,
        goal_pt: np.ndarray,
        args: PlanningOptions,
        random_sampler: Sampler,
    ):
        self.num_dtrees = num_dtrees
        self.init_energy()
        self.particles: List[DisjointTreeParticle] = []
        self.local_samplers_to_be_rstart: List[DisjointTreeParticle] = []
        self.goal_pt = goal_pt
        self.random_sampler = random_sampler
        self.args = args

    def add_to_restart(self, lsampler: DisjointTreeParticle):
        if lsampler not in self.local_samplers_to_be_rstart:
            self.local_samplers_to_be_rstart.append(lsampler)

    def init_energy(self):
        """ """
        self.dtrees_energy = np.ones(self.num_dtrees) * ENERGY_START
        self.resync_prob()

    def modify_energy(self, idx, factor=None, set_val=None):
        """Modify the energy of the given particle

        :param idx: index of the target particle
        :param factor: scalar factor
        :param set_val: if given, set the value.

        """
        # keep track how much energy this operation would modify,
        # so we can change the energy_sum accordingly
        old_energy = self.dtrees_energy[idx]
        if set_val is not None:
            self.dtrees_energy[idx] = set_val
        elif factor is not None:
            self.dtrees_energy[idx] *= factor
        else:
            raise ValueError("Nothing set in modify_energy")

        delta = self.dtrees_energy[idx] - old_energy
        self.cur_energy_sum += delta

    def confirm(self, idx: int, pos: np.ndarray):
        self.particles[idx].confirm(pos)

    def new_pos(self, idx: int, pos: np.ndarray, dir):
        self.particles[idx].provision_dir = dir

    def get_pos(self, idx):
        return self.particles[idx].pos

    def get_prob(self):
        return self.dtrees_energy / self.cur_energy_sum

    def resync_prob(self):
        self.dtrees_energy = np.nan_to_num(self.dtrees_energy)
        if self.dtrees_energy.sum() < 1e-10:
            # particle_energy demonlished to 0...
            # work around to add energy all particles
            self.dtrees_energy[:] = 1
        self.cur_energy_sum = self.dtrees_energy.sum()

    def new_pos_in_free_space(self):
        """Return a new position in free space."""
        Stats.get_instance().lsampler_restart_counter += 1
        while True:

            new_p = self.random_sampler.get_next_pos()[0]

            Stats.get_instance().add_sampled_node(new_p)
            if not self.args.engine.cc.feasible(new_p):
                Stats.get_instance().add_invalid(obs=True)
            else:
                Stats.get_instance().add_free()
                break
        return new_p


def pygame_rrdt_sampler_paint_init(sampler):
    """Visualisation init function for rrdt sampler

    :param sampler: sampler to be visualised

    """
    import pygame

    sampler.particles_layer = pygame.Surface(
        (
            sampler.args.engine.upper[0] * sampler.args.scaling,
            sampler.args.engine.upper[1] * sampler.args.scaling,
        ),
        pygame.SRCALPHA,
    )


def pygame_rrdt_sampler_paint(sampler):
    """Visualisation function for rrdt sampler

    :param sampler: sampler to be visualised

    """

    def get_color_transists(value, max_prob, min_prob):
        """

        :param value:
        :param max_prob:
        :param min_prob:

        """
        denominator = max_prob - min_prob
        if denominator == 0:
            denominator = 1  # prevent division by zero
        return 220 - 180 * (1 - (value - min_prob) / denominator)

    if sampler._last_prob is None:
        return
    max_num = sampler._last_prob.max()
    min_num = sampler._last_prob.min()
    for i, p in enumerate(sampler.p_manager.particles):
        sampler.particles_layer.fill((255, 128, 255, 0))
        # get a transition from green to red
        c = get_color_transists(sampler._last_prob[i], max_num, min_num)
        c = max(min(255, c), 50)
        color = (c, c, 0)
        sampler.args.env.draw_circle(
            pos=p.pos, colour=color, radius=4, layer=sampler.particles_layer
        )
        sampler.args.env.window.blit(sampler.particles_layer, (0, 0))


def pygame_rrdt_planner_paint(planner):
    planner.args.env.path_layers.fill(Colour.ALPHA_CK)

    drawn_nodes_pairs = set()
    # Draw disjointed trees
    for tree in planner._disjointed_trees:
        bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
        while bfs.has_next():
            newnode = bfs.next()
            for e in newnode.edges:
                new_set = frozenset({newnode, e})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)
                    planner.args.env.draw_path(newnode, e)
    # Draw root tree
    for n in planner.root.nodes:
        if n.parent is not None:
            new_set = frozenset({n, n.parent})
            if new_set not in drawn_nodes_pairs:
                drawn_nodes_pairs.add(new_set)
                planner.args.env.draw_path(n, n.parent, Colour.orange)
    planner.visualiser.draw_solution_path()


def klampt_rrdt_planner_paint(planner):
    drawn_nodes_pairs = set()

    def generate_random_colors():
        import colorsys
        import ghalton

        perms = ghalton.EA_PERMS[:1]
        sequencer = ghalton.GeneralizedHalton(perms)
        while True:
            x = sequencer.get(1)[0][0]
            HSV_tuple = (x, 1, 0.6)
            rgb_colour = colorsys.hsv_to_rgb(*HSV_tuple)
            yield (*rgb_colour, 1.0)  # add alpha channel

    color_gen = generate_random_colors()

    # Draw disjointed trees
    for tree in planner._disjointed_trees:
        c = next(color_gen)
        # draw nodes
        for node in tree.nodes:
            planner.args.env.draw_node(
                planner.args.engine.cc.get_eef_world_pos(node.pos), colour=c
            )
        # draw edges
        bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
        while bfs.has_next():
            newnode = bfs.next()
            for e in newnode.edges:
                new_set = frozenset({newnode, e})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)

                    planner.args.env.draw_path(
                        planner.args.engine.cc.get_eef_world_pos(newnode.pos),
                        planner.args.engine.cc.get_eef_world_pos(e.pos),
                        colour=c,
                    )
    # Draw root tree
    c = next(color_gen)
    # override to red
    c = (1, 0, 0, 1)
    # draw nodes
    for node in planner.root.nodes:
        planner.args.env.draw_node(
            planner.args.engine.cc.get_eef_world_pos(node.pos), colour=c
        )
    # draw edges
    for n in planner.root.nodes:
        if n.parent is not None:
            new_set = frozenset({n, n.parent})
            if new_set not in drawn_nodes_pairs:
                drawn_nodes_pairs.add(new_set)
                planner.args.env.draw_path(
                    planner.args.engine.cc.get_eef_world_pos(n.pos),
                    planner.args.engine.cc.get_eef_world_pos(n.parent.pos),
                    colour=c,
                )


# start register
sampler_id = "rrdt_sampler"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=RRdTSampler,
    visualise_pygame_paint=pygame_rrdt_sampler_paint,
    visualise_pygame_paint_init=pygame_rrdt_sampler_paint_init,
)

planner_registry.register_planner(
    "rrdt",
    planner_class=RRdTPlanner,
    visualise_pygame_paint=pygame_rrdt_planner_paint,
    visualise_klampt_paint=klampt_rrdt_planner_paint,
    sampler_id=sampler_id,
)
# finish register
