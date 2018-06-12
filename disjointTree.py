from checkCollision import *

JOIN_TREES_RADIUS = 10

class TreesManager:
    def __init__(self):
        self.root = TreeRoot(particle_handler=None)
        self.disjointTrees = []
        pass

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
        cloest_node = cloest_tree = cloest_dist = None
        for tree in [*self.disjointTrees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            for n in tree.nodes:
                _dist = dist(node.pos, n.pos)
                if _dist < radius:
                    if cloest_dist is None:
                        cloest_dist = _dist
                    if _dist <= cloest_dist:
                        cloest_node = n
                        cloest_tree = tree
        return cloest_node, cloest_tree


    def join_tree_to_root(self, tree, kill_particle=True):
        """It will join the given tree to the root"""
        # remove tree2 from disjoint tree list
        raise Exception("NOT implemented yet")

    def join_trees(self, tree1, tree2, kill_particle=True):
        """
        Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.
        """
        # TODO make it more general to return a list of nodes within radius..?
        if tree1 is self.root:
            self.join_tree_to_root(tree1)
            return
        if tree2 is self.root:
            self.join_tree_to_root(tree2)
            return
        tree1.nodes.extend(tree2.nodes)
        if kill_particle:
            tree2.particle_handler = None
        # remove tree2 from disjoint tree list
        self.disjointTrees.remove(tree2)

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
RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 3

class DisjointTreeParticle(Particle):
    def __init__(self, tree_manager, direction=None, pos=None, root=False):
        super().__init__(direction=direction, pos=pos)
        # create a new node at the newly spawned location
        if root:
            self.tree = TreeRoot(particle_handler=self)
            tree_manager.root = self.tree
        else:
            self.tree = TreeDisjoint(particle_handler=self)
            tree_manager.disjointTrees.append(self.tree)
        self.tree.nodes.append(Node(pos))


class DisjointParticleFilterSampler(ParticleFilterSampler):

    def init(self, **kwargs):
        self.tree_manager = kwargs['tree_manager']
        super().init(**kwargs)

        # ditch the particles created by the original particle filter sampler, and
        # create ones that has link towards the disjointed tree
        self.p_manager.particles = []
        for _ in range(self.p_manager.num_particles):
            pos = self.p_manager.new_pos_in_free_space()
            dt_p = DisjointTreeParticle(
                direction=random.uniform(0, math.pi * 2),
                pos=pos,
                tree_manager=self.tree_manager)

            self.p_manager.particles.append(dt_p)

        # Monkey patch the RRT for this smapler's specific stuff
        kwargs['RRT'].run = lambda x=kwargs['RRT'] : rrt_dt_patched_run(x)

    def particles_random_free_space_restart(self):
        tmp = []
        for i in range(self.p_manager.size()):
            if self.p_manager.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                print(self.p_manager.particles[i].pos)
                tmp.append(self.p_manager.particles_energy[i])
                randomPt = self.p_manager.new_pos_in_free_space()
                self.p_manager.particles[i] = DisjointTreeParticle(pos=randomPt, tree_manager=self.tree_manager)
                self.p_manager.modify_energy(i, set_val=ENERGY_START)
                print(self.p_manager.particles[i].pos)
        return tmp

    def restart_specific_particle(self, particle):
        """Restart the given particle in a new free space"""
        self.p_manager.particles.remove(particle)
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

        p = self.randomWalk(choice)

        self.last_particle = p
        return (p, self.p_manager.particles[choice].tree,
                lambda c=choice, **kwargs: self.report_success(c, **kwargs),
                lambda c=choice, **kwargs: self.report_fail(c, **kwargs))


############################################################
##    PATCHING RRT with disjointed-tree specific stuff    ##
############################################################

class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0
        self.edges = []

def rrt_dt_patched_run(self):
    self.fpsClock.tick(10000)
    def findNearestNeighbour(node):
        nn = self.nodes[0]
        for p in self.nodes:
            if dist(p.pos, node.pos) < dist(nn.pos, node.pos):
                nn = p
        return nn

    from rrtstar import Colour, GOAL_RADIUS
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
            self.nodes.append(newnode)
            parent_tree.nodes.append(newnode)
            # check and see if the new node can be connected with other existing tree
            cloest_node, cloest_tree = self.tree_manager.find_nearest_node_from_neighbour(node=newnode,
                                                                     parent_tree=parent_tree,
                                                                     radius=self.RADIUS)
            if cloest_node is not None and self.cc.path_is_free(newnode, cloest_node):
                print("JOINted!")
                pygame.draw.line(self.path_layers, Colour.black, newnode.pos*self.SCALING, cloest_node.pos*self.SCALING, self.SCALING)
                cloest_node.edges.append(newnode)
                newnode.edges.append(cloest_node)
                self.sampler.restart_specific_particle(parent_tree.particle_handler)
                self.tree_manager.join_trees(cloest_tree, parent_tree)

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

    def chooseLeastCostParent(self, nn, newnode):
        for p in self.nodes:
            if(self.cc.pathIsFree(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and
               p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos)):
                nn = p
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return newnode, nn

    def reWire(self, newnode):
        for i in range(len(self.nodes)):
            p = self.nodes[i]
            if(p != newnode.parent and self.cc.pathIsFree(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and newnode.cost + dist(p.pos, newnode.pos) < p.cost):
                # draw over the old wire
                pygame.draw.line(self.path_layers, Colour.white, p.pos*self.SCALING, p.parent.pos*self.SCALING, self.SCALING)
                # update new parents (re-wire)
                p.parent = newnode
                p.cost = newnode.cost + dist(p.pos, newnode.pos)
                pygame.draw.line(self.path_layers, Colour.black, p.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)
