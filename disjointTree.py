from checkCollision import *
from overrides import overrides

JOIN_TREES_RADIUS = 10

def update_progress(progress, total_num, num_of_blocks=10):
    percentage = progress/total_num
    print('\r[{bar:<{num_of_blocks}}] {cur}/{total} {percen:0.1f}%'.format(
        bar='#'*int(percentage * num_of_blocks),
        cur=progress,
        total=total_num,
        percen=percentage*100,
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
            next_node = self.next_node_to_visit.pop(-1)
            if next_node not in self.visitedNodes and next_node in self.validNodes:
                break
            if len(self.next_node_to_visit) < 1:
                return False
        self.visit_node(next_node)
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

    def find_nearest_node(self, node, parent_tree):
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
                # only add the cloest node form each trees
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
        # add all nodoes from disjoint tree via rrt star method
        total_num = len(tree.nodes)
        progress = 0
        print("> Joining to root tree")
        while bfs.has_next():
            newnode = bfs.next()
            if newnode not in tree.nodes:
                print(tree.nodes)
                print(hex(id(tree.nodes)))
                print(bfs.validNodes)
                print(hex(id(bfs.validNodes)))
                print(newnode)
                raise Exception("WTF???")
            progress += 1
            update_progress(progress, total_num, num_of_blocks=20)
            # draw white (remove edge for visual) on top of disjointed tree
            for e in (x for x in newnode.edges if x not in bfs.visitedNodes):
                pygame.draw.line(self.rrt.path_layers, Colour.white, e.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            newnode, nn = rrt_star_add_node(self.rrt, newnode)
            self.root.nodes.append(newnode)
            pygame.draw.line(self.rrt.path_layers, Colour.black, nn.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            # remove this node's edges (as we don't have a use on them anymore) to free memory
            del newnode.edges




            total = 0
            for t in self.disjointedTrees:
                if newnode in t.nodes:
                    total += 1

            assert total == 1, "HUH??? in more than one tree"

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

        print(" => Joining trees with size {} to {}".format(len(tree1.nodes), len(tree2.nodes)))
        if tree1 is self.root or tree2 is self.root:
            # find the disjointed tree
            if tree1 is not self.root:
                tree1, tree2 = tree2, tree1
            # find which middle_node belongs to the disjointed tree
            middle_node = tree1_node if tree1_node in tree2.nodes else tree2_node
            self.join_tree_to_root(tree2, middle_node)
        else:
            # kill of tree2's particle if both tree has particle alive
            # otherwise, keep whichever the particle that is still alive
            if tree1.particle_handler is None:
                # swap
                tree1, tree2 = tree2, tree1
            tree1.nodes.extend(tree2.nodes)
        del tree2.nodes
        self.disjointedTrees.remove(tree2)
        if tree2.particle_handler is not None:
            tree2.particle_handler.restart()
        self.rrt.connect_two_nodes(tree1_node, tree2_node, tree1, perform_rrt_star=False, add_edges_only=True)
        return tree1

############################################################
##              Disjointed Particles Sampler              ##
############################################################
import random
import pygame
import math
from pygame.locals import *
from particleFilterSampler import ParticleFilterSampler, Particle

from particleFilterSampler import RANDOM_RESTART_PARTICLES_ENERGY_UNDER, ENERGY_START
RANDOM_RESTART_EVERY = 20
RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 1.75

class DisjointTreeParticle(Particle):
    @overrides
    def __init__(self, tree_manager, p_manager, add_pos_to_existing_tree, direction=None, pos=None, isroot=False, startPtNode=None):
        self.isroot = isroot
        self.p_manager = p_manager
        self.add_pos_to_existing_tree = add_pos_to_existing_tree
        self.tree_manager = tree_manager
        if isroot:
            self.tree = self.tree_manager.root
            self.tree.nodes.append(startPtNode)
        super().__init__(direction=direction, pos=pos)

    @overrides
    def restart(self, direction=None, pos=None):
        if self.isroot:
            return self.init_as_root_tree(direction, pos)

        # remove tree reference to his particle
        try:
            self.tree.particle_handler = None
        except AttributeError:
            # probably this is its first init
            pass
        if pos is None:
            # get random position
            while True:
                pos = self.p_manager.new_pos_in_free_space()
                if not self.add_pos_to_existing_tree(pos):
                    break
        self.p_manager.modify_energy(particle_ref=self, set_val=ENERGY_START)
        self.tree = TreeDisjoint(particle_handler=self)
        self.tree.nodes.append(Node(pos))
        self.tree_manager.disjointedTrees.append(self.tree)
        self.tree.particle_handler = self
        super().restart(direction, pos)

    def init_as_root_tree(self, direction, pos):
        self.isroot = False
        self.tree.particle_handler = self
        super().restart(direction, pos)

class DisjointParticleFilterSampler(ParticleFilterSampler):

    def init(self, **kwargs):
        # Monkey patch the RRT for this smapler's specific stuff
        import types
        kwargs['RRT'].run = types.MethodType(rrt_dt_patched_run, kwargs['RRT'])
        kwargs['RRT'].connect_two_nodes = types.MethodType(connect_two_nodes, kwargs['RRT'])

        self.tree_manager = kwargs['tree_manager']
        super().init(**kwargs)
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
                add_pos_to_existing_tree=self.add_pos_to_existing_tree,
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
                                add_pos_to_existing_tree=self.add_pos_to_existing_tree,
                                ))


    def add_pos_to_existing_tree(self, pos):
        """Try to add pos to existing tree. If success, return True."""
        from rrtstar import Colour
        newnode = Node(pos)
        nearest_nodes = self.tree_manager.find_nearest_node_from_neighbour(
                node=newnode,
                parent_tree=None,
                radius=self.RRT.RADIUS)
        merged = False
        parent_tree = None
        for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
             # check and see if the new node can be connected with other existing tree
             # When it starts, it is an orphan node so simply join it to any other tree. When it is trying to
             # join another tree, we then need to join tree with tree.
            if self.RRT.cc.path_is_free(newnode, nearest_neighbour_node):
                if parent_tree is None:
                    # joining the orphan node to a tree
                    self.RRT.connect_two_nodes(newnode, nearest_neighbour_node, nearest_neighbour_tree)
                    parent_tree = nearest_neighbour_tree
                    print(" ==> Joining to existing tree with size {}".format(len(nearest_neighbour_tree.nodes)))
                else:
                    # joining a tree with tree
                    parent_tree = self.tree_manager.join_trees(parent_tree, nearest_neighbour_tree,
                                                 tree1_node=newnode, tree2_node=nearest_neighbour_node)
                self.RRT.update_screen(ignore_redraw_paths=True)
                merged = True
                # return merged
        return merged
    def particles_random_free_space_restart(self):
        tmp = []
        for i in range(self.p_manager.size()):
            if self.p_manager.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                tmp.append(self.p_manager.particles_energy[i])

                self.p_manager.particles[i].restart()
        return tmp


    def get_next_node(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1

        if self._c_random > RANDOM_RESTART_EVERY > 0:
            _p = self.particles_random_free_space_restart()
            if _p:
                print("Rand restart at counter {}, with p {}".format(self.counter, _p))
            self._c_random = 0
        # get a node to random walk
        prob = self.p_manager.get_prob()
        self._last_prob = prob  # this will be used to paint particles
        try:
            choice = np.random.choice(range(self.p_manager.size()), p=prob)
        except ValueError as e:
            # NOTE dont know why the probability got out of sync... We notify the use, then try re-sync the prob
            print("!! probability got exception '{}'... trying to re-sync prob again.".format(e))
            self.p_manager.resync_prob()
            self._last_prob = prob
            choice = np.random.choice(range(self.p_manager.size()), p=prob)

        # NOTE This controls if testing (via mouse) or actual runs
        pos = self.randomWalk(choice)
        # pos, choice = self.randomWalk_by_mouse()

        self.last_particle = pos
        return (pos, self.p_manager.particles[choice].tree,
                lambda c=choice, **kwargs: self.report_success(c, **kwargs),
                lambda c=choice, **kwargs: self.report_fail(c, **kwargs))

    def randomWalk_by_mouse(self):
        """FOR testing purpose. Mimic random walk, but do so via mouse click."""
        import mouseSampler
        mouse = mouseSampler.MouseSampler()
        mouse.init(SCALING=self.scaling)
        pos = mouse.get_mouse_click_position()
        # find the cloest particle from this position
        _dist = None
        p_idx = None
        for i in range(len(self.p_manager.particles)):
            p = self.p_manager.particles[i]
            if _dist is None or _dist > dist(pos, p.pos):
                _dist = dist(pos, p.pos)
                p_idx = i
        print("num of tree: {}".format(len(self.tree_manager.disjointedTrees)))
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
        self.cost = 0
        self.edges = []

def rrt_star_add_node(rrt_instance, newnode, nn=None):
    """This function perform finding optimal parent, and rewiring."""
    from rrtstar import Colour, GOAL_RADIUS
    self = rrt_instance
    def choose_least_cost_parent(newnode, nn=None):
        """Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)"""
        if nn is not None:
            _newnode_to_nn_cost = dist(newnode.pos, nn.pos)
        for p in self.tree_manager.root.nodes:
            if p is nn:
                continue # avoid unnecessary computations
            _newnode_to_p_cost = dist(newnode.pos, p.pos)
            if self.cc.path_is_free(newnode, p) and _newnode_to_p_cost < self.RADIUS:
               # This is another valid parent. Check if it's better than our current one.
               if nn is None or (p.cost + _newnode_to_p_cost < nn.cost + _newnode_to_nn_cost):
                nn = p
                _newnode_to_nn_cost = _newnode_to_p_cost
        if nn is None:
            raise Exception("ERROR: Provided nn=None, and cannot find any valid nn by this function. This newnode is not close to the root tree...?")
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return newnode, nn
    def rewire(newnode):
        for n in self.tree_manager.root.nodes:
            if(n != newnode.parent and self.cc.path_is_free(n, newnode) and
               dist(n.pos, newnode.pos) < self.RADIUS and newnode.cost + dist(n.pos, newnode.pos) < n.cost):
                # draw over the old wire
                pygame.draw.line(self.path_layers, Colour.white, n.pos*self.SCALING, n.parent.pos*self.SCALING, self.SCALING)
                # update new parents (re-wire)
                n.parent = newnode
                n.cost = newnode.cost + dist(n.pos, newnode.pos)
                pygame.draw.line(self.path_layers, Colour.black, n.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)
    newnode, nn = choose_least_cost_parent(newnode, nn=nn)
    rewire(newnode)
    # check for goal condition
    if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
        if newnode.cost < self.c_max:
            self.c_max = newnode.cost
            self.goalPt.parent = newnode
    return newnode, nn

def connect_two_nodes(self, newnode, nn, parent_tree=None, perform_rrt_star=True, add_edges_only=False):
    """Add node to disjoint tree OR root tree. Draw line for it too."""
    from rrtstar import Colour
    pygame.draw.line(self.path_layers, Colour.black, newnode.pos*self.SCALING, nn.pos*self.SCALING, self.SCALING)
    if parent_tree is self.tree_manager.root:
        if perform_rrt_star:
            # using rrt* algorithm to add each nodes
            newnode, nn = rrt_star_add_node(self, newnode, nn)
    else:
        newnode.edges.append(nn)
        nn.edges.append(newnode)
    if not add_edges_only:
        parent_tree.nodes.append(newnode)
    return newnode, nn


def rrt_dt_patched_run(self):
    from rrtstar import Colour, GOAL_RADIUS
    self.fpsClock.tick(10000)

    while self.stats.valid_sample < self.NUMNODES:
        rand = None
        # Get an sample that is free (not in blocked space)
        while rand is None or self.collides(rand.pos):
            coordinate, parent_tree, report_success, report_fail = self.sampler.get_next_node()
            rand = Node(coordinate)
            self.stats.add_sampled_node(rand)
        nn = self.tree_manager.find_nearest_node(rand, parent_tree)
        # get an intermediate node according to step-size
        newnode = Node(self.step_from_to(nn.pos, rand.pos))
        # check if it is free or not ofree
        if not self.cc.path_is_free(nn, newnode):
            self.sampler.add_sample(p=rand, free=False)
            self.stats.add_invalid(perm=False)
            report_fail()
        else:
            report_success(pos=newnode.pos)
            self.stats.add_free()
            x, y = newnode.pos
            ######################
            newnode, nn = self.connect_two_nodes(newnode, nn, parent_tree)

            # self.nodes.append(newnode)
            nearest_nodes = self.tree_manager.find_nearest_node_from_neighbour(
                     node=newnode,
                     parent_tree=parent_tree,
                     radius=self.RADIUS)
            pygame.draw.line(self.path_layers, Colour.black, newnode.pos*self.SCALING, nn.pos*self.SCALING, self.SCALING)
            i = 0
            # Check to see if root exists in the nearest tree. If so, add it at the last
            for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
                 # check and see if the new node can be connected with other existing tree
                if self.cc.path_is_free(newnode, nearest_neighbour_node):
                    parent_tree = self.tree_manager.join_trees(parent_tree, nearest_neighbour_tree,
                                                               tree1_node=newnode, tree2_node=nearest_neighbour_node)


            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")
        self.update_screen()

    self.wait_for_exit()

            #
            #
            # for e in (x for x in newnode.edges if x not in bfs.visitedNodes):
            #     pygame.draw.line(self.rrt.path_layers, Colour.white, e.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            # nn = self.find_nearest_node(newnode, self.root)
            # nn, newnode = rrt_star_add_node(self.rrt, nn, newnode)
            # self.root.nodes.append(newnode)
            # pygame.draw.line(self.rrt.path_layers, Colour.black, nn.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            # # remove this node's edges (as we don't have a use on them anymore) to free memory
            # newnode.edges = None
    # pygame.display.update()

############################################################
##                         Classes                        ##
############################################################


class TreeRoot:
    def __init__(self, particle_handler):
        self.particle_handler = particle_handler
        self.nodes = []

class TreeDisjoint(TreeRoot):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)



##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
