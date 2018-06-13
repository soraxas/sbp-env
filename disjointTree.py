from checkCollision import *

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
    def __init__(self, node):
        self.visitedNodes = []
        self.next_node_to_visit = [node]
        self.next_node = None
    def visit_node(self, node):
        self.visitedNodes.append(node)
        self.next_node_to_visit.extend(node.edges)
    def has_next(self):
        if self.next_node is not None:
            return True
        if len(self.next_node_to_visit) < 1:
            return False
        # get next available node
        while True:
            next_node = self.next_node_to_visit.pop(-1)
            if next_node not in self.visitedNodes:
                break
            if len(self.next_node_to_visit) < 1:
                return False
        self.visit_node(next_node)
        self.next_node = next_node
        return True
    def next(self):
        node = self.next_node
        self.next_node = None
        return node
class TreesManager:
    def __init__(self, RRT):
        self.root = TreeRoot(particle_handler=self)
        self.disjointTrees = []
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
        Return the cloest node (and its corresponding tree) within radius that is not from the given tree (from other neighbourhood trees)
        Return None if none exists
        """
        # TODO make it more general to return a list of nodes within radius..?
        nearest_neighbour_node = nearest_neighbour_tree = nearest_dist = None
        for tree in [*self.disjointTrees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            for n in tree.nodes:
                _dist = dist(node.pos, n.pos)
                if _dist < radius:
                    if nearest_dist is None:
                        nearest_dist = _dist
                    if _dist <= nearest_dist:
                        nearest_neighbour_node = n
                        nearest_neighbour_tree = tree
        return nearest_neighbour_node, nearest_neighbour_tree


    def join_tree_to_root(self, tree, middle_node):
        """It will join the given tree to the root"""
        from rrtstar import Colour
        bfs = BFS(middle_node)
        bfs.visitedNodes.extend(self.root.nodes)
        # add all nodoes from disjoint tree via rrt star method
        total_num = len(tree.nodes)
        progress = 0
        print("> Joining to root tree")
        while bfs.has_next():
            newnode = bfs.next()
            progress += 1
            update_progress(progress, total_num, num_of_blocks=20)
            # draw white (remove edge for visual) on top of disjointed tree
            for e in (x for x in newnode.edges if x not in bfs.visitedNodes):
                pygame.draw.line(self.rrt.path_layers, Colour.white, e.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            nn = self.find_nearest_node(newnode, self.root)
            nn, newnode = rrt_star_add_node(self.rrt, nn, newnode)
            self.root.nodes.append(newnode)
            pygame.draw.line(self.rrt.path_layers, Colour.black, nn.pos*self.rrt.SCALING, newnode.pos*self.rrt.SCALING, self.rrt.SCALING)
            # remove this node's edges (as we don't have a use on them anymore) to free memory
            newnode.edges = None

        # raise Exception("NOT implemented yet")

    def join_trees(self, tree1, tree2, restart_tree_particle_func, middle_node1=None, middle_node2=None):
        """
        Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.

        middle_node1 & 2 represent the nodes that join the two tree together. It only matters currently to
        joining root tree to disjointed treeself.

        Return the tree that has not been killed
        """
        # TODO make it more general to return a list of nodes within radius..?
        if tree1 is self.root or tree2 is self.root:
            # find the disjointed tree
            tree = tree2 if tree2 is not self.root else tree1
            # find which middle_node belongs to the disjointed tree
            middle_node = middle_node1 if middle_node1 in tree.nodes else middle_node2
            self.join_tree_to_root(tree, middle_node)
            restart_tree_particle_func(tree)
            self.disjointTrees.remove(tree)
            return True
        else:
            # kill of tree2's particle if both tree has particle alive
            # otherwise, keep whichever the particle that is still alive
            if tree1.particle_handler.dead:
                # swap
                tree1, tree2 = tree2, tree1
            tree1.nodes.extend(tree2.nodes)
            self.disjointTrees.remove(tree2)
            restart_tree_particle_func(tree2)
            return False
        # return true if both particles should be removed. return false if onlt tree1's particle should be removed.

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
    def __init__(self, tree_manager, direction=None, pos=None, root=False, startPtNode=None):
        super().__init__(direction=direction, pos=pos)
        # create a new node at the newly spawned location
        if root:
            self.tree = tree_manager.root
            self.tree.particle_handler = self
            self.tree.nodes.append(startPtNode)
        else:
            self.tree = TreeDisjoint(particle_handler=self)
            tree_manager.disjointTrees.append(self.tree)
            self.tree.nodes.append(Node(pos))
        self.dead = False


class DisjointParticleFilterSampler(ParticleFilterSampler):

    def add_pos_to_existing_tree(self, pos):
        """Try to add pos to existing tree. If success, return True."""
        from rrtstar import Colour
        newnode = Node(pos)
        nearest_neighbour_node, nearest_neighbour_tree = self.tree_manager.find_nearest_node_from_neighbour(node=newnode,
                                                                 parent_tree=None,
                                                                 radius=self.RRT.RADIUS)
         # check and see if the new node can be connected with other existing tree
        if nearest_neighbour_node is not None and self.RRT.cc.path_is_free(newnode, nearest_neighbour_node):
            print(nearest_neighbour_node)
            pygame.draw.line(self.RRT.path_layers, Colour.black, newnode.pos*self.RRT.SCALING, nearest_neighbour_node.pos*self.RRT.SCALING, self.RRT.SCALING)
            newnode.edges.append(nearest_neighbour_node)
            nearest_neighbour_node.edges.append(newnode)
            print(" ===> Joining existing tree with size {}".format(len(nearest_neighbour_tree.nodes)))
            return True
        return False


    def init(self, **kwargs):
        self.tree_manager = kwargs['tree_manager']
        super().init(**kwargs)
        # ditch the particles created by the original particle filter sampler, and
        # create ones that has link towards the disjointed tree
        self.p_manager.particles = []
        for _ in range(self.p_manager.num_particles - 1):
            # check if this free space is in close approximate with existing tree, just joint it to that tree
            # and re-try spawning a new particle
            while True:
                pos = self.p_manager.new_pos_in_free_space()
                if not self.add_pos_to_existing_tree(pos):
                    # This denotes we can now spawn a new particle (that is not close to existing trees)
                    break


            dt_p = DisjointTreeParticle(
                direction=random.uniform(0, math.pi * 2),
                pos=pos,
                tree_manager=self.tree_manager)

            self.p_manager.particles.append(dt_p)
        # spawn one that comes from the root
        self.p_manager.particles.append(
            DisjointTreeParticle(direction=random.uniform(0, math.pi * 2),
                                pos=self.startPt,
                                root=True,
                                startPtNode=self.RRT.startPt,
                                tree_manager=self.tree_manager))

        # Monkey patch the RRT for this smapler's specific stuff
        kwargs['RRT'].run = lambda x=kwargs['RRT'] : rrt_dt_patched_run(x)

    def particles_random_free_space_restart(self):
        tmp = []
        for i in range(self.p_manager.size()):
            if self.p_manager.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                tmp.append(self.p_manager.particles_energy[i])
                randomPt = self.p_manager.new_pos_in_free_space()
                self.p_manager.particles[i].dead = True
                self.p_manager.particles[i] = DisjointTreeParticle(pos=randomPt, tree_manager=self.tree_manager)
                self.p_manager.modify_energy(i, set_val=ENERGY_START)
        return tmp

    def restart_specific_tree_particle(self, tree):
        """Restart the given particle in a new free space"""
        if tree.particle_handler.dead:
            # ignore
            return
        self.p_manager.particles.remove(tree.particle_handler)
        tree.particle_handler.dead = True
        self.p_manager.particles.append(DisjointTreeParticle(pos=self.p_manager.new_pos_in_free_space(),
                                                             tree_manager=self.tree_manager))

    def get_next_node(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1

        if self._c_random > RANDOM_RESTART_EVERY and RANDOM_RESTART_EVERY > 0:
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
        print("num of tree: {}".format(len(self.tree_manager.disjointTrees)))
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

def rrt_star_add_node(rrt_instance, nn, newnode):
    """This function perform finding optimal parent, and rewiring."""
    from rrtstar import Colour, GOAL_RADIUS
    self = rrt_instance
    def choose_least_cost_parent(nn, newnode):
        """Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)"""
        for p in self.tree_manager.root.nodes:
            if p is nn:
                continue # avoid unnecessary computations
            if(self.cc.path_is_free(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and
               p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos)):
                nn = p
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return nn, newnode
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
    nn, newnode = choose_least_cost_parent(nn, newnode)
    rewire(newnode)
    # check for goal condition
    if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
        if newnode.cost < self.c_max:
            self.c_max = newnode.cost
    return nn, newnode

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
            if parent_tree is self.tree_manager.root:
                # using rrt* algorithm to add each nodes
                nn, newnode = rrt_star_add_node(self, nn, newnode)
            else:
                nn.edges.append(newnode)
                newnode.edges.append(nn)

            self.nodes.append(newnode)
            parent_tree.nodes.append(newnode)
            nearest_neighbour_node, nearest_neighbour_tree = self.tree_manager.find_nearest_node_from_neighbour(node=newnode,
                                                                     parent_tree=parent_tree,
                                                                     radius=self.RADIUS)

             # check and see if the new node can be connected with other existing tree
            if nearest_neighbour_node is not None and self.cc.path_is_free(newnode, nearest_neighbour_node):
                print(" => Joining tree with size {} to {}".format(len(parent_tree.nodes), len(nearest_neighbour_tree.nodes)))
                pygame.draw.line(self.path_layers, Colour.black, newnode.pos*self.SCALING, nearest_neighbour_node.pos*self.SCALING, self.SCALING)

                if not self.tree_manager.join_trees(nearest_neighbour_tree, parent_tree, middle_node1=newnode, middle_node2=nearest_neighbour_node,
                                                restart_tree_particle_func=self.sampler.restart_specific_tree_particle):
                    # this functino return true if it is joining disjoint tree to root tree
                    # We only add edges if it is not joining to root
                    newnode.edges.append(nearest_neighbour_node)
                    nearest_neighbour_node.edges.append(newnode)

            # rewire to see what the newly added node can do for us
            # self.rewire(newnode)
            pygame.draw.line(self.path_layers, Colour.black, nn.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)
            if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
                if newnode.cost < self.c_max:
                    print("REACHED DRAW SOLUTION PATH")
                    # self.c_max = newnode.cost
                    # self.draw_solution_path()

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
