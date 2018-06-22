#!/usr/bin/env python

# rrtstar.py
# This program generates a
# asymptotically optimal rapidly exploring random tree (RRT*) in a rectangular region.

import sys
import random
import math
import pygame
import logging
from pygame.locals import *
from math import sqrt, cos, sin, atan2
import numpy as np
from matplotlib import pyplot as plt

import disjointTree as dt
from checkCollision import *

LOGGER = logging.getLogger(__name__)

# constants
INF = float('inf')
ALPHA_CK = 255,0,255

GOAL_RADIUS = 10
GOAL_BIAS = 0.005

class Colour:
    white = 255, 255, 255
    black = 20, 20, 40
    red = 255, 0, 0
    blue = 0, 0, 255
    green = 0,150,0
    cyan = 20,200,200
    orange = 255, 160, 16

class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0  # index 0 is x, index 1 is y
        self.parent = None
        self.children = []

class SampledNodes:
    def __init__(self, p):
        self.pos = p
        self.framedShowed = 0

class stats:
    def __init__(self):
        self.invalid_sample_temp = 0
        self.invalid_sample_perm = 0
        self.valid_sample = 0
        self.sampledNodes = []

    def add_invalid(self,perm):
        if perm:
            self.invalid_sample_perm += 1
        else:
            self.invalid_sample_temp += 1

    def add_free(self):
        self.valid_sample += 1

    def add_sampled_node(self, node, not_a_node=False):
        if not_a_node:
            node = Node(node)
        self.sampledNodes.append(SampledNodes(node.pos.astype(int)))



############################################################

class RRT:
    def __init__(self, showSampledPoint, scaling, image, epsilon, max_number_nodes, radius,
                 sampler, goalBias=True, ignore_step_size=False, always_refresh=False,
                 disable_pygame=False):
        # initialize and prepare screen
        pygame.init()
        self.stats = stats()
        self.img = pygame.image.load(image)
        self.cc = CollisionChecker(self.img)
        self.XDIM = self.img.get_width()
        self.YDIM = self.img.get_height()
        self.SCALING = scaling
        self.always_refresh = always_refresh

        self.EPSILON = epsilon
        self.NUMNODES = max_number_nodes
        self.RADIUS = radius
        self.fpsClock = pygame.time.Clock()
        self.goalBias = goalBias
        self.showSampledPoint = showSampledPoint
        self.ignore_step_size = ignore_step_size

        self.c_max = INF

        pygame.display.set_caption('RRTstar')
        # screen.fill(white)
        ################################################################################
        # text
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Arial', 15 * self.SCALING)
        ################################################################################
        # main window
        self.window = pygame.display.set_mode([self.XDIM * self.SCALING, self.YDIM * self.SCALING])
        ################################################################################
        # background aka the room
        self.background = pygame.Surface( [self.XDIM, self.YDIM] )
        self.background.blit(self.img,(0,0))
        # resize background to match windows
        self.background = pygame.transform.scale(self.background, [self.XDIM * self.SCALING, self.YDIM * self.SCALING])
        ################################################################################
        # path of RRT*
        self.path_layers = pygame.Surface( [self.XDIM * self.SCALING, self.YDIM * self.SCALING] )
        self.path_layers.fill(ALPHA_CK)
        self.path_layers.set_colorkey(ALPHA_CK)
        ################################################################################
        # layers to store the solution path
        self.solution_path_screen = pygame.Surface( [self.XDIM * self.SCALING, self.YDIM * self.SCALING] )
        self.solution_path_screen.fill(ALPHA_CK)
        self.solution_path_screen.set_colorkey(ALPHA_CK)
        ################################################################################
        # layers to store the sampled points
        self.sampledPoint_screen = pygame.Surface( [self.XDIM * self.SCALING, self.YDIM * self.SCALING] )
        self.sampledPoint_screen.fill(ALPHA_CK)
        self.sampledPoint_screen.set_colorkey(ALPHA_CK)
        ################################################################################
        self.tree_manager = dt.TreesManager(RRT=self)
        self.nodes = []
        self.sampledNodes = []

        self.startPt = None
        self.goalPt = None

        self.sampler = sampler
        ##################################################
        # Get starting and ending point
        LOGGER.info('Select Starting Point and then Goal Point')
        self.fpsClock.tick(10)
        while self.startPt is None or self.goalPt is None:
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mousePos = (int(e.pos[0] / self.SCALING), int(e.pos[1] / self.SCALING))
                    if self.startPt is None:
                        if not self.collides(mousePos):
                            LOGGER.info(('starting point set: ' + str(mousePos)))
                            self.startPt = Node(mousePos)
                            self.nodes.append(self.startPt)

                    elif self.goalPt is None:
                        if not self.collides(mousePos):
                            LOGGER.info(('goal point set: ' + str(mousePos)))
                            self.goalPt = Node(mousePos)
                    elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        LOGGER.info("Leaving.")
                        return
            self.update_screen(update_all=True)

        ##################################################
        # calculate information regarding shortest path
        self.c_min = dist(self.startPt.pos, self.goalPt.pos)
        self.x_center = (self.startPt.pos[0]+self.goalPt.pos[0])/2 , (self.startPt.pos[1]+self.goalPt.pos[1])/2
        dy = self.goalPt.pos[1] - self.startPt.pos[1]
        dx = self.goalPt.pos[0] - self.startPt.pos[0]
        self.angle = math.atan2(-dy, dx)

        self.sampler.init(RRT=self, XDIM=self.XDIM, YDIM=self.YDIM, SCALING=self.SCALING, EPSILON=self.EPSILON,
                          startPt=self.startPt.pos, goalPt=self.goalPt.pos, tree_manager=self.tree_manager)

    ############################################################

    def collides(self, p):
        """check if point is white (which means free space)"""
        x = int(p[0])
        y = int(p[1])
        # make sure x and y is within image boundary
        if(x < 0 or x >= self.img.get_width() or
           y < 0 or y >= self.img.get_height()):
            return True
        color = self.img.get_at((x, y))
        pointIsObstacle = (color != pygame.Color(*Colour.white))
        return pointIsObstacle

    def step_from_to(self,p1, p2):
        """Get a new point from p1 to p2, according to step size."""
        if self.ignore_step_size:
            return p2
        if dist(p1, p2) < self.EPSILON:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            pos = p1[0] + self.EPSILON * cos(theta), p1[1] + self.EPSILON * sin(theta)
            return pos

    def choose_least_cost_parent(self, newnode, nn=None, nodes=None):
        """Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)"""
        if nn is not None:
            _newnode_to_nn_cost = dist(newnode.pos, nn.pos)
        for p in nodes:
            if p is nn:
                continue  # avoid unnecessary computations
            _newnode_to_p_cost = dist(newnode.pos, p.pos)
            if _newnode_to_p_cost < self.RADIUS and self.cc.path_is_free(newnode, p):
                # This is another valid parent. Check if it's better than our current one.
                if nn is None or (p.cost + _newnode_to_p_cost < nn.cost + _newnode_to_nn_cost):
                    nn = p
                    _newnode_to_nn_cost = _newnode_to_p_cost
        if nn is None:
            raise Exception(
                "ERROR: Provided nn=None, and cannot find any valid nn by this function. This newnode is not close to the root tree...?")
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        nn.children.append(newnode)

        return newnode, nn


    def rewire(self, newnode, nodes, already_rewired=None):
        """Reconsider parents of nodes that had change, so that the optimiality would change instantly"""
        if len(nodes) < 1:
            return
        if already_rewired is None:
            already_rewired = {newnode}
        # for n in nodes:
        for n in (x for x in nodes if x not in already_rewired):
            _newnode_to_n_cost = dist(n.pos, newnode.pos)
            if (n != newnode.parent and _newnode_to_n_cost < self.RADIUS and
                    self.cc.path_is_free(n, newnode) and newnode.cost + _newnode_to_n_cost < n.cost):
                # draw over the old wire
                self.draw_path(n, n.parent, Colour.white)
                reconsider = (n.parent, *n.children)
                n.parent.children.remove(n)
                n.parent = newnode
                newnode.children.append(n)
                n.cost = newnode.cost + _newnode_to_n_cost
                already_rewired.add(n)
                self.draw_path(n, newnode, Colour.blue)
                self.rewire(n, reconsider, already_rewired=already_rewired)


    def run(self):
        self.fpsClock.tick(10000)
        def find_nearest_neighbour(node):
            nn = self.nodes[0]
            for p in self.nodes:
                if dist(p.pos, node.pos) < dist(nn.pos, node.pos):
                    nn = p
            return nn

        while self.stats.valid_sample < self.NUMNODES:

            # Get an sample that is free (not in blocked space)
            while True:
                coordinate, report_success, report_fail = self.sampler.get_next_node()
                rand = Node(coordinate)
                self.stats.add_sampled_node(rand)
                if not self.collides(rand.pos):
                    break
                report_fail(pos=rand, obstacle=True)
                self.stats.add_invalid(perm=True)
            # Found a node that is not in X_obs

            nn = find_nearest_neighbour(rand)
            # get an intermediate node according to step-size
            newnode = Node(self.step_from_to(nn.pos, rand.pos))
            # check if it has a free path to nn or not
            if not self.cc.path_is_free(nn, newnode):
                self.stats.add_invalid(perm=False)
                report_fail(pos=rand, free=False)
            else:
                self.stats.add_free()
                self.sampler.add_tree_node(newnode.pos)
                report_success(pos=newnode.pos, nn=nn, rand=rand)

                ######################
                # consider not only the length but also the current shortest cost
                newnode, nn = self.choose_least_cost_parent(newnode, nn, nodes=self.nodes)
                self.nodes.append(newnode)
                # rewire to see what the newly added node can do for us
                self.rewire(newnode, self.nodes)
                self.draw_path(nn, newnode)

                if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
                    self.c_max = newnode.cost
                    self.goalPt.parent = newnode
                    newnode.children.append(self.goalPt.parent)
                    self.draw_solution_path()

                self.process_pygame_event()
            self.update_screen()

    def draw_path(self, node1, node2, colour=Colour.black, line_modifier=1):
        pygame.draw.line(self.path_layers, colour,
                         node1.pos * self.SCALING,
                         node2.pos * self.SCALING,
                         line_modifier * self.SCALING)

    @staticmethod
    def process_pygame_event():
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                LOGGER.info("Leaving.")
                sys.exit(0)

    @staticmethod
    def wait_for_exit():
        while True:
            RRT.process_pygame_event()


############################################################
##                    DRAWING RELATED                     ##
############################################################

    def draw_solution_path(self):
        if self.c_max == INF:
            # nothing to d
            return
        # redraw new path
        self.solution_path_screen.fill(ALPHA_CK)
        nn = self.goalPt.parent
        self.c_max = nn.cost
        while nn != self.startPt:
            self.draw_path(nn, nn.parent, colour=Colour.green, line_modifier=5)
            nn = nn.parent
        self.window.blit(self.path_layers,(0,0))
        self.window.blit(self.solution_path_screen,(0,0))
        pygame.display.update()

    def update_screen(self, update_all=False):
        if 'refresh_cnt' not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.always_refresh:
            count = 0 #FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

###################################################################################

        if count % 80 == 0:
            self.redraw_paths()

        ##### Solution path
        if count % 50 == 0:
            self.draw_solution_path()
            # self.wait_for_exit()
        # limites the screen update
        if count % 10 == 0:
            self.window.blit(self.background,(0,0))

        ##### Tree paths
        if count % 10 == 0:
            self.window.blit(self.path_layers,(0,0))
            self.window.blit(self.solution_path_screen,(0,0))
            if self.startPt is not None:
                pygame.draw.circle(self.path_layers, Colour.cyan, self.startPt.pos*self.SCALING, GOAL_RADIUS*self.SCALING)
            if self.goalPt is not None:
                pygame.draw.circle(self.path_layers, Colour.blue, self.goalPt.pos*self.SCALING, GOAL_RADIUS*self.SCALING)

        ##### Sampler hook
        if count % 10 == 0:
            try:
                self.sampler.paint(self.window)
            except AttributeError:
                pass

        ##### Sampled points
        if count % 2 == 0:
            show_sampled_point_for = 1
            self.sampledPoint_screen.fill(ALPHA_CK)
            # Draw sampled nodes
            sampledNodes = self.stats.sampledNodes
            for i in reversed(range(len(sampledNodes))):
                pygame.draw.circle(self.sampledPoint_screen, Colour.red, sampledNodes[i].pos*self.SCALING, 2*self.SCALING)
                sampledNodes[i].framedShowed += 1

                if sampledNodes[i].framedShowed >= show_sampled_point_for:
                    sampledNodes.pop(i)
            self.window.blit(self.sampledPoint_screen,(0,0))

        ##### Texts
        if count % 10 == 0:
            _cost = 'INF' if self.c_max == INF else round(self.c_max, 2)
            if 'DisjointParticleFilterSampler' in self.sampler.__str__():
                num_nodes = sum(len(tree.nodes) for tree in (*self.tree_manager.disjointedTrees, self.tree_manager.root))
            else:
                num_nodes = len(self.nodes)
            text = 'Cost_min: {}  | Nodes: {}'.format(_cost, num_nodes)
            self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.YDIM * self.SCALING * 0.88))
            text = 'Invalid sample: {}(temp) {}(perm)'.format(self.stats.invalid_sample_temp, self.stats.invalid_sample_perm)
            self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.YDIM * self.SCALING * 0.95))

        pygame.display.update()

    def redraw_paths(self):
        from disjointTree import BFS
        # these had already been drawn
        drawn_nodes_pairs = set()
        self.path_layers.fill(ALPHA_CK)
        if 'DisjointParticleFilterSampler' in self.sampler.__str__():
            # Draw disjointed trees
            for tree in self.tree_manager.disjointedTrees:
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.draw_path(newnode, e)
            # Draw root tree
            for n in self.tree_manager.root.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.draw_path(n, n.parent, Colour.orange)
        else:
            # Draw path trees
            for n in self.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.draw_path(n, n.parent)

        if self.startPt is not None:
            pygame.draw.circle(self.path_layers, Colour.cyan, self.startPt.pos*self.SCALING, GOAL_RADIUS*self.SCALING)
        if self.goalPt is not None:
            pygame.draw.circle(self.path_layers, Colour.blue, self.goalPt.pos*self.SCALING, GOAL_RADIUS*self.SCALING)
