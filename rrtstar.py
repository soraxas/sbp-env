#!/usr/bin/env python

# rrtstar.py
# This program generates a
# asymptotically optimal rapidly exploring random tree (RRT*) in a rectangular region.

import sys
import random
import math
import pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2
import numpy as np
from matplotlib import pyplot as plt

import disjointTree as dt
from checkCollision import *

# constants
INF = sys.maxsize
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

class Node:
    pos = None  # index 0 is x, index 1 is y
    cost = 0
    parent = None
    def __init__(self, pos):
        self.pos = np.array(pos)

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

    def add_sampled_node(self, node):
        self.sampledNodes.append(SampledNodes(node.pos.astype(int)))



############################################################

class RRT:
    def __init__(self, showSampledPoint, scaling, image, epsilon, max_number_nodes, radius, sampler, goalBias=True, ignore_step_size=False, always_refresh=False):
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
        # # probability layer
        # self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * self.SCALING,self.PROB_BLOCK_SIZE * self.SCALING), pygame.SRCALPHA)
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
        self.tree_manager = dt.TreesManager()
        self.nodes = []
        self.sampledNodes = []

        self.startPt = None
        self.goalPt = None

        self.sampler = sampler
        ##################################################
        # Get starting and ending point
        print('Select Starting Point and then Goal Point')
        self.fpsClock.tick(10)
        while self.startPt is None or self.goalPt is None:
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mousePos = (int(e.pos[0] / self.SCALING), int(e.pos[1] / self.SCALING))
                    if self.startPt is None:
                        if self.collides(mousePos,initialSetup=True) == False:
                            print(('starting point set: ' + str(mousePos)))
                            self.startPt = Node(mousePos)
                            self.nodes.append(self.startPt)

                    elif self.goalPt is None:
                        if self.collides(mousePos,initialSetup=True) == False:
                            print(('goal point set: ' + str(mousePos)))
                            self.goalPt = Node(mousePos)
                    elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving.")
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

    def collides(self, p, initialSetup=False):
        """check if point is white (which means free space)"""
        x = int(p[0])
        y = int(p[1])
        # make sure x and y is within image boundary
        if(x < 0 or x >= self.img.get_width() or
           y < 0 or y >= self.img.get_height()):
            return True
        color = self.img.get_at((x, y))
        pointIsObstacle = (color != pygame.Color(*Colour.white))
        if not initialSetup:
            self.sampler.add_sample(p=p, obstacle=pointIsObstacle)
        if pointIsObstacle:
            self.stats.add_invalid(perm=pointIsObstacle)
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

    def get_least_cost_parent(self, nn, newnode):
        for p in self.nodes:
            if(self.cc.path_is_free(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and
               p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos)):
                nn = p
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return newnode, nn

    def rewire(self, newnode):
        for i in range(len(self.nodes)):
            p = self.nodes[i]
            if(p != newnode.parent and self.cc.path_is_free(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and newnode.cost + dist(p.pos, newnode.pos) < p.cost):
                # draw over the old wire
                pygame.draw.line(self.path_layers, Colour.white, p.pos*self.SCALING, p.parent.pos*self.SCALING, self.SCALING)
                # update new parents (re-wire)
                p.parent = newnode
                p.cost = newnode.cost + dist(p.pos, newnode.pos)
                pygame.draw.line(self.path_layers, Colour.black, p.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)

    def run(self):
        self.fpsClock.tick(10000)
        goal_bias_success = False
        def findNearestNeighbour(node):
            nn = self.nodes[0]
            for p in self.nodes:
                if dist(p.pos, node.pos) < dist(nn.pos, node.pos):
                    nn = p
            return nn

        while self.stats.valid_sample < self.NUMNODES:

            rand = None
            # Get an sample that is free (not in blocked space)
            while rand is None or self.collides(rand.pos):
                coordinate, report_success, report_fail = self.sampler.get_next_node()
                rand = Node(coordinate)
                self.stats.add_sampled_node(rand)
            nn = findNearestNeighbour(rand)
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
                self.sampler.add_tree_node(x, y)
                if rand is not None and ' Sampler.add_sample_line ' not in self.sampler.add_sample_line.__str__():
                    # only run this if the sampler wants (had implemented) this method
                    # add all in between point of nearest node of the random pt as valid
                    x1, y1 = self.cc.get_coor_before_collision(nn, rand)
                    self.sampler.add_sample_line(x, y, x1, y1)
                ######################
                # consider not only the length but also the current shortest cost
                [newnode, nn] = self.get_least_cost_parent(nn, newnode)
                self.nodes.append(newnode)
                # rewire to see what the newly added node can do for us
                self.rewire(newnode)
                pygame.draw.line(self.path_layers, Colour.black, nn.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)

                if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
                    if newnode.cost < self.c_max:
                        self.c_max = newnode.cost
                        self.draw_solution_path()

                for e in pygame.event.get():
                    if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving.")
            self.update_screen()

        self.wait_for_exit()

    @staticmethod
    def wait_for_exit():
        while True:
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")

############################################################
##                    DRAWING RELATED                     ##
############################################################

    def draw_solution_path(self):
        if self.c_max == INF:
            # nothing to d
            return
        # redraw new path
        self.solution_path_screen.fill(ALPHA_CK)
        nn = self.nodes[0]
        for p in self.nodes:
            if dist(p.pos, self.goalPt.pos) < dist(nn.pos, self.goalPt.pos):
                nn = p
        while nn != self.startPt:
            pygame.draw.line(self.solution_path_screen, Colour.green, nn.pos*self.SCALING, nn.parent.pos*self.SCALING, 5*self.SCALING)
            nn = nn.parent
        self.window.blit(self.path_layers,(0,0))
        self.window.blit(self.solution_path_screen,(0,0))
        pygame.display.update()

    def update_screen(self, update_all=False):
        if 'refresh_cnt' not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0
        update_all
        if update_all or self.always_refresh:
            count = 0 #FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

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
        if count % 5 == 0:
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
            text = 'Cost_min: {}  | Nodes: {}'.format(_cost, len(self.nodes))
            self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.YDIM * self.SCALING * 0.88))
            text = 'Invalid sample: {}(temp) {}(perm)'.format(self.stats.invalid_sample_temp, self.stats.invalid_sample_perm)
            self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.YDIM * self.SCALING * 0.95))

        pygame.display.update()
