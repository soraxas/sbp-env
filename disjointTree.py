import random
import math
import sys
import logging

from checkCollision import *
from overrides import overrides

LOGGER = logging.getLogger(__name__)

JOIN_TREES_RADIUS = 10

def update_progress(progress, total_num, num_of_blocks=10):
    if not logging.getLogger().isEnabledFor(logging.INFO):
        return
    percentage = progress / total_num
    print('\r[{bar:<{num_of_blocks}}] {cur}/{total} {percen:0.1f}%'.format(
        bar='#' * int(percentage * num_of_blocks),
        cur=progress,
        total=total_num,
        percen=percentage * 100,
        num_of_blocks=num_of_blocks), end='')
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


class TreesManager:
    def __init__(self, RRT):
        self.root = TreeRoot(particle_handler=self)
        self.disjointedTrees = []
        self.rrt = RRT

    @staticmethod
    def find_nearest_node(node, parent_tree):
        """
        Find nearest node from given node
        """
        nn = parent_tree.nodes[0]
        for p in parent_tree.nodes:
            if dist(p.pos, node.pos) < dist(nn.pos, node.pos):
                nn = p
        return nn

    def find_nearest_node_from_neighbour(self, node, parent_tree, radius):
        """
        Given a tree, a node within that tree, and radius
        Return a list of cloest nodes (and its corresponding tree) within the radius (that's from other neighbourhood trees)
        Return None if none exists
        IF root exists in the list, add it at the last position (So the connection behaviour would remain stable)
            This ensure all previous action would only add add edges to each nodes, and only the last action would it
            modifies the entire tree structures wtih rrt* procedures.
        """
        nearest_nodes = {}
        for tree in [*self.disjointedTrees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            for n in tree.nodes:
                # only add the closest node form each trees
                _dist = dist(node.pos, n.pos)
                if _dist < radius:
                    if tree in nearest_nodes and nearest_nodes[tree][0] < _dist:
                        # this node is not closer than other found nodes.
                        continue
                    nearest_nodes[tree] = (_dist, n)
        # construct list of the found solution. And root at last (or else the result won't be stable)
        root = nearest_nodes.pop(self.root, None)
        nearest_nodes_list = [(nearest_nodes[key][1], key) for key in nearest_nodes]
        if root is not None:
            nearest_nodes_list.append((root[1], self.root))
        return nearest_nodes_list

    def join_tree_to_root(self, tree, middle_node):
        """It will join the given tree to the root"""
        from rrtstar import Colour
        bfs = BFS(middle_node, validNodes=tree.nodes)
        # add all nodes from disjoint tree via rrt star method
        total_num = len(tree.nodes)
        progress = 0
        LOGGER.info("> Joining to root tree")
        while bfs.has_next():
            newnode = bfs.next()
            progress += 1
            update_progress(progress, total_num, num_of_blocks=20)
            # draw white (remove edge for visual) on top of disjointed tree
            for e in (x for x in newnode.edges if x not in bfs.visitedNodes and x in bfs.validNodes):
                self.rrt.draw_path(e, newnode, Colour.white)
            self.rrt.connect_two_nodes(newnode, nn=None, parent_tree=self.root)
            # remove this node's edges (as we don't have a use on them anymore) to free memory
            del newnode.edges

        assert progress == total_num, "Inconsistency in BFS walk {} != {}".format(progress, total_num)

        # raise Exception("NOT implemented yet")

    def join_trees(self, tree1, tree2, tree1_node, tree2_node):
        """
        Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.

        tree1_node & 2 represent the nodes that join the two tree together. It only matters currently to
        joining root tree to disjointed treeself.

        Return the tree that has not been killed
        """
        assert tree1 is not tree2, "Both given tree should not be the same"
        if tree1 not in self.disjointedTrees:
            assert tree1 is self.root, "Given tree is neither in disjointed tree, nor is it the root: {}".format(tree1)
        if tree2 not in self.disjointedTrees:
            assert tree2 is self.root, "Given tree is neither in disjointed tree, nor is it the root: {}".format(tree2)

        LOGGER.info(" => Joining trees with size {} to {}".format(len(tree1.nodes), len(tree2.nodes)))
        # Re-arrange only. Make it so that tree1 will always be root (if root exists among the two)
        # And tree1 node must always be belong to tree1, tree2 node belong to tree2
        if tree1 is not self.root:
            # set tree1 as root (if root exists among the two)
            tree1, tree2 = tree2, tree1
        if tree1_node in tree2.nodes or tree2_node in tree1.nodes:
            # swap to correct position
            tree1_node, tree2_node = tree2_node, tree1_node
        assert tree1_node in tree1.nodes, "Given nodes does not belong to the two given corresponding trees"
        assert tree2_node in tree2.nodes, "Given nodes does not belong to the two given corresponding trees"

        if tree1 is self.root:
            # find which middle_node belongs to the disjointed tree
            self.join_tree_to_root(tree2, tree2_node)
            self.rrt.connect_two_nodes(tree1_node, tree2_node, draw_only=True)
        else:
            self.rrt.connect_two_nodes(tree1_node, tree2_node)
            tree1.nodes.extend(tree2.nodes)
        del tree2.nodes
        self.disjointedTrees.remove(tree2)
        if tree2.particle_handler is not None:
            tree2.particle_handler.restart()
        return tree1


############################################################
##              Disjointed Particles Sampler              ##
############################################################
from particleFilterSampler import ParticleFilterSampler, Particle, RANDOM_RESTART_PARTICLES_ENERGY_UNDER, ENERGY_START

RANDOM_RESTART_EVERY = 20
ENERGY_START = 10
RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 0.75


class DisjointTreeParticle(Particle):
    @overrides
    def __init__(self, tree_manager, p_manager, direction=None, pos=None, isroot=False,
                 startPtNode=None):
        self.isroot = isroot
        self.p_manager = p_manager
        self.tree_manager = tree_manager
        if isroot:
            self.tree = self.tree_manager.root
            self.tree.nodes.append(startPtNode)
        super().__init__(direction=direction, pos=pos)

    @overrides
    def restart(self, direction=None, pos=None):
        if self.isroot:
            # root particles has a different initialisation method
            # (for the first time)
            self.isroot = False
            self.tree.particle_handler = self
            super().restart(direction, pos)
            return
        try:
            # remove tree reference to his particle
            self.tree.particle_handler = None
        except AttributeError:
            # probably this is its first init
            pass
        if pos is None:
            # get random position
            pos = self.p_manager.new_pos_in_free_space()
            if self.p_manager.rrt.add_pos_to_existing_tree(Node(pos), None):
                # we need to abort the restart procedure. add this to pending restart
                self.p_manager.add_to_restart(self)
                # Successfully found a new valid node that's close to existing tree
                # Return true to indicate it (and abort restart for now)
                return False
        # initialise to initial value, create new d-tree
        self.p_manager.modify_energy(particle_ref=self, set_val=ENERGY_START)
        self.tree = TreeDisjoint(particle_handler=self)
        self.tree.nodes.append(Node(pos))
        self.tree_manager.disjointedTrees.append(self.tree)
        self.tree.particle_handler = self
        super().restart(direction, pos)
        return True



class DisjointParticleFilterSampler(ParticleFilterSampler):

    @overrides
    def init(self, **kwargs):

        super().init(**kwargs)
        # Monkey patch the RRT for this smapler's specific stuff
        import types

        self.RRT.run_once = types.MethodType(rrt_dt_patched_run_once, self.RRT)
        self.RRT.connect_two_nodes = types.MethodType(connect_two_nodes, self.RRT)
        self.RRT.add_pos_to_existing_tree = types.MethodType(add_pos_to_existing_tree, self.RRT)

        self.lsamplers_to_be_restart = []
        self.tree_manager = kwargs['tree_manager']
        # ditch the particles created by the original particle filter sampler, and
        # create ones that has link towards the disjointed tree
        self.p_manager.particles = []
        for _ in range(self.p_manager.num_particles - 1):
            pos = self.p_manager.new_pos_in_free_space()

            dt_p = DisjointTreeParticle(
                direction=random.uniform(0, math.pi * 2),
                pos=pos,
                tree_manager=self.tree_manager,
                p_manager=self.p_manager,
            )

            self.p_manager.particles.append(dt_p)
        # spawn one that comes from the root
        self.p_manager.particles.append(
            DisjointTreeParticle(direction=random.uniform(0, math.pi * 2),
                                 pos=self.startPt,
                                 isroot=True,
                                 startPtNode=self.RRT.startPt,
                                 tree_manager=self.tree_manager,
                                 p_manager=self.p_manager,
                                 ))

    def particles_random_free_space_restart(self):
        for i in range(self.p_manager.size()):
            if self.p_manager.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                self.p_manager.add_to_restart(self.p_manager.particles[i])

    @overrides
    def report_success(self, idx, **kwargs):
        self.p_manager.confirm(idx, kwargs['pos'])
        self.p_manager.modify_energy(idx=idx, factor=0.99)

    @overrides
    def get_valid_next_node(self):
        """Loop until we find a valid next node"""
        while True:
            _tmp = self.get_next_node()
            if _tmp is None:
                # This denotes a particle had tried to restart and added the new node
                # to existing tree instead. Skip remaining steps and iterate to next loop
                return None
            coordinate, parent_tree, report_success, report_fail = _tmp
            rand = Node(coordinate)
            self.RRT.stats.add_sampled_node(rand)
            if not self.RRT.collides(rand.pos):
                return rand, parent_tree, report_success, report_fail
            report_fail(pos=rand, obstacle=True)
            self.RRT.stats.add_invalid(perm=True)

    @overrides
    def get_next_node(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1

        if self._c_random > RANDOM_RESTART_EVERY > 0:
            self._c_random = 0
            self.particles_random_free_space_restart()
        if not self.p_manager.restart_all_pending_local_samplers():
            LOGGER.debug("Adding node to existing trees.")
            return None
        # get a node to random walk
        choice = self.get_random_choice()

        # NOTE This controls if testing (via mouse) or actual runs
        pos = self.randomWalk(choice)
        # pos, choice = self.randomWalk_by_mouse()

        self.last_particle = pos
        return (pos, self.p_manager.particles[choice].tree,
                lambda c=choice, **kwargs: self.report_success(c, **kwargs),
                lambda c=choice, **kwargs: self.report_fail(c, **kwargs))

    def randomWalk_by_mouse(self):
        """FOR testing purpose. Mimic random walk, but do so via mouse click."""
        from mouseSampler import MouseSampler as mouse
        pos = mouse.get_mouse_click_position(scaling=self.scaling)
        # find the cloest particle from this position
        _dist = None
        p_idx = None
        for i in range(len(self.p_manager.particles)):
            p = self.p_manager.particles[i]
            if _dist is None or _dist > dist(pos, p.pos):
                _dist = dist(pos, p.pos)
                p_idx = i
        LOGGER.debug("num of tree: {}".format(len(self.tree_manager.disjointedTrees)))
        self.p_manager.new_pos(idx=p_idx,
                               pos=pos,
                               dir=0)
        return pos, p_idx


############################################################
##    PATCHING RRT with disjointed-tree specific stuff    ##
############################################################

class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0  # index 0 is x, index 1 is y
        self.edges = []
        self.children = []
    def __repr__(self):
        try:
            num_edges = len(self.edges)
        except AttributeError:
            num_edges = "DELETED"
        return "Node(pos={}, cost={}, num_edges={})".format(self.pos, self.cost, num_edges)


def rrt_star_add_node(rrt_instance, newnode, nn=None):
    """This function perform finding optimal parent, and rewiring."""
    from rrtstar import Colour, GOAL_RADIUS
    self = rrt_instance

    newnode, nn = self.choose_least_cost_parent(newnode, nn=nn, nodes=self.tree_manager.root.nodes)
    self.rewire(newnode, nodes=self.tree_manager.root.nodes)
    # check for goal condition
    if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
        if newnode.cost < self.c_max:
            self.c_max = newnode.cost
            self.goalPt.parent = newnode
            newnode.children.append(self.goalPt.parent)
    return newnode, nn


def connect_two_nodes(self, newnode, nn, parent_tree=None, draw_only=False):
    """Add node to disjoint tree OR root tree. Draw line for it too."""
    from rrtstar import Colour
    if not draw_only:
        if parent_tree is self.tree_manager.root:
            # using rrt* algorithm to add each nodes
            newnode, nn = rrt_star_add_node(self, newnode, nn)
        else:
            newnode.edges.append(nn)
            nn.edges.append(newnode)
        if parent_tree is not None:
            parent_tree.nodes.append(newnode)
    self.draw_path(newnode, nn)
    return newnode, nn

def add_pos_to_existing_tree(self, newnode, parent_tree):
    """Try to add pos to existing tree. If success, return True."""
    merged = False
    nearest_nodes = self.tree_manager.find_nearest_node_from_neighbour(
        node=newnode,
        parent_tree=parent_tree,
        radius=self.RADIUS)
    for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
        if self.cc.path_is_free(newnode, nearest_neighbour_node):
            if parent_tree is None:
                ### joining ORPHAN NODE to a tree
                self.connect_two_nodes(newnode, nearest_neighbour_node, nearest_neighbour_tree)
                parent_tree = nearest_neighbour_tree
                LOGGER.debug(" ==> During respawning particle, joining to existing tree with size: {}".format(len(nearest_neighbour_tree.nodes)))
            else:
                ### joining a TREE to another tree
                try:
                    parent_tree = self.tree_manager.join_trees(parent_tree, nearest_neighbour_tree,
                                                           tree1_node=newnode, tree2_node=nearest_neighbour_node)
                except AssertionError as e:
                    LOGGER.exception("== Assertion error in joining sampled point to existing tree... Skipping this node...")
            merged = True
    return merged

def rrt_dt_patched_run_once(self):
    # Get an sample that is free (not in blocked space)
    _tmp = self.sampler.get_valid_next_node()
    if _tmp is None:
        # we have added a new samples when respawning a local sampler
        return
    rand, parent_tree, report_success, report_fail = _tmp

    nn = self.tree_manager.find_nearest_node(rand, parent_tree)
    # get an intermediate node according to step-size
    newnode = Node(self.step_from_to(nn.pos, rand.pos))
    # check if it is free or not ofree
    if not self.cc.path_is_free(nn, newnode):
        self.stats.add_invalid(perm=False)
        report_fail(pos=rand, free=False)
    else:
        self.stats.add_free()
        self.sampler.add_tree_node(newnode.pos)
        report_success(pos=newnode.pos)
        ######################
        newnode, nn = self.connect_two_nodes(newnode, nn, parent_tree)
        # try to add this newnode to existing trees
        self.add_pos_to_existing_tree(newnode, parent_tree)


############################################################
##                         Classes                        ##
############################################################


class TreeRoot:
    def __init__(self, particle_handler):
        self.particle_handler = particle_handler
        self.nodes = []
    def __repr__(self):
        string = super().__repr__()
        string += '\n'
        import pprint
        string += pprint.pformat(vars(self), indent=4)
        # string += ', '.join("%s: %s" % item for item in vars(self).items())
        return string


class TreeDisjoint(TreeRoot):
    @overrides
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
