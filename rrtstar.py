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


from checkCollision import *

# constants
INFINITE = sys.maxsize
ALPHA_CK = 255,0,255

GOAL_RADIUS = 10

SCALING = 2

GOAL_BIAS = 0.005


##################################################
## COLOR ##
white = 255, 255, 255
black = 20, 20, 40
red = 255, 0, 0
blue = 0, 0, 255
green = 0,150,0
##################################################


class Node:
    pos = None  # index 0 is x, index 1 is y
    cost = 0
    parent = None

    def __init__(self, pos):
        self.pos = pos

class stats:
    def __init__(self):
        self.invalid_sample_temp = 0
        self.invalid_sample_perm = 0
        self.valid_sample = 0

    def addInvalid(self,perm):
        if perm:
            self.invalid_sample_perm += 1
        else:
            self.invalid_sample_temp += 1

    def addFree(self):
        self.valid_sample += 1


############################################################

from matplotlib import pyplot as plt

class RRT:

    def __init__(self, image, epsilon, max_number_nodes, radius, sampler, goalBias=True, check_entire_path=True):
        # initialize and prepare screen
        pygame.init()
        self.stats = stats()
        self.img = pygame.image.load(image)
        self.XDIM = self.img.get_width()
        self.YDIM = self.img.get_height()

        self.EPSILON = epsilon
        self.NUMNODES = max_number_nodes
        self.RADIUS = radius
        self.fpsClock = pygame.time.Clock()
        self.goalBias = goalBias

        self.c_max = INFINITE
        self.check_entire_path = check_entire_path

        pygame.display.set_caption('RRTstar')
        # screen.fill(white)
        ################################################################################
        # text
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Arial', 15 * SCALING)
        ################################################################################
        # main window
        self.window = pygame.display.set_mode([self.XDIM * SCALING, self.YDIM * SCALING])
        ################################################################################
        # # probability layer
        # self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * SCALING,self.PROB_BLOCK_SIZE * SCALING), pygame.SRCALPHA)
        ################################################################################
        # background aka the room
        self.background = pygame.Surface( [self.XDIM, self.YDIM] )
        self.background.blit(self.img,(0,0))
        # resize background to match windows
        self.background = pygame.transform.scale(self.background, [self.XDIM * SCALING, self.YDIM * SCALING])
        ################################################################################
        # path of RRT*
        self.path_layers = pygame.Surface( [self.XDIM * SCALING, self.YDIM * SCALING] )
        self.path_layers.fill(ALPHA_CK)
        self.path_layers.set_colorkey(ALPHA_CK)
        # rescale to make it bigger
        self.path_layers_rescale = pygame.Surface( [self.XDIM * SCALING, self.YDIM * SCALING] )
        self.path_layers_rescale.fill(ALPHA_CK)
        self.path_layers_rescale.set_colorkey(ALPHA_CK)
        ################################################################################
        # layers to store the solution path
        self.solution_path_screen = pygame.Surface( [self.XDIM * SCALING, self.YDIM * SCALING] )
        self.solution_path_screen.fill(ALPHA_CK)
        self.solution_path_screen.set_colorkey(ALPHA_CK)
        # rescale to make it bigger
        self.solution_path_screen_rescale = pygame.Surface( [self.XDIM * SCALING, self.YDIM * SCALING] )
        self.solution_path_screen_rescale.fill(ALPHA_CK)
        self.solution_path_screen_rescale.set_colorkey(ALPHA_CK)
        ################################################################################
        self.nodes = []

        self.startPt = None
        self.goalPt = None
        ##################################################
        # Get starting and ending point
        print('Select Starting Point and then Goal Point')
        self.fpsClock.tick(10)
        while self.startPt is None or self.goalPt is None:
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mousePos = (int(e.pos[0] / SCALING), int(e.pos[1] / SCALING))
                    if self.startPt is None:
                        if self.collides(mousePos,initialSetup=True) == False:
                            print(('starting point set: ' + str(mousePos)))
                            self.startPt = Node(np.array(mousePos))
                            self.nodes.append(self.startPt)

                    elif self.goalPt is None:
                        if self.collides(mousePos,initialSetup=True) == False:
                            print(('goal point set: ' + str(mousePos)))
                            self.goalPt = Node(np.array(mousePos))
                    elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving.")
            self.update_screen(0)
        ##################################################
        # calculate information regarding shortest path
        self.c_min = dist(self.startPt.pos, self.goalPt.pos)
        self.x_center = (self.startPt.pos[0]+self.goalPt.pos[0])/2 , (self.startPt.pos[1]+self.goalPt.pos[1])/2
        dy = self.goalPt.pos[1] - self.startPt.pos[1]
        dx = self.goalPt.pos[0] - self.startPt.pos[0]
        self.angle = math.atan2(-dy, dx)

        self.sampler = sampler
        self.sampler.init(RRT=self, XDIM=self.XDIM, YDIM=self.YDIM, SCALING=SCALING)

    ############################################################

    def collides(self, p, initialSetup=False):    #check if point is white (which means free space)
        # make sure x and y is within image boundary
        x = int(p[0])
        y = int(p[1])
        if x < 0 or x >= self.img.get_width() or y < 0 or y >= self.img.get_height():
            return True
        color = self.img.get_at((x, y))
        # print(color)
        if color == pygame.Color(*white):
            pointIsObstacle = False
        else:
            pointIsObstacle = True
        if not initialSetup:
            self.sampler.addSample(p=p, obstacle=pointIsObstacle)
        if pointIsObstacle:
            self.stats.addInvalid(perm=pointIsObstacle)
        return pointIsObstacle


    def step_from_to(self,p1, p2):
        if dist(p1, p2) < self.EPSILON:
            return np.array(p2)
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            pos = p1[0] + self.EPSILON * cos(theta), p1[1] + self.EPSILON * sin(theta)
            return np.array(pos)


    def chooseParent(self,nn, newnode):
        for p in self.nodes:
            if checkIntersect(p, newnode, self.img) and dist(p.pos, newnode.pos) < self.RADIUS and p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos):
                nn = p
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return newnode, nn


    def reWire(self, newnode):
        for i in range(len(self.nodes)):
            p = self.nodes[i]
            if checkIntersect(p, newnode, self.img) and p != newnode.parent and dist(p.pos, newnode.pos) < self.RADIUS and newnode.cost + dist(p.pos, newnode.pos) < p.cost:
                pygame.draw.line(self.path_layers, white, p.pos*SCALING, p.parent.pos*SCALING, SCALING)
                p.parent = newnode
                p.cost = newnode.cost + dist(p.pos, newnode.pos)
                self.nodes[i] = p
                pygame.draw.line(self.path_layers, black, p.pos*SCALING, newnode.pos*SCALING, SCALING)

    def drawSolutionPath(self):
        # redraw new path
        self.solution_path_screen.fill(ALPHA_CK)

        nn = self.nodes[0]
        for p in self.nodes:
            if dist(p.pos, self.goalPt.pos) < dist(nn.pos, self.goalPt.pos):
                nn = p
        while nn != self.startPt:
            pygame.draw.line(self.solution_path_screen, green, nn.pos*SCALING, nn.parent.pos*SCALING, 5*SCALING)
            nn = nn.parent
        self.window.blit(self.path_layers,(0,0))
        self.window.blit(self.solution_path_screen,(0,0))
        pygame.display.update()


    def run(self):

        ##################################################

        self.fpsClock.tick(10000)
        goal_bias_success = False
        # for i in range(self.NUMNODES):
        while self.stats.valid_sample < self.NUMNODES:
            # probabiilty to bias toward goal (while not reaching goal yet)
            if self.c_max != INFINITE:
                self.drawSolutionPath()
                self.wait_for_exit()
                pass

            if self.c_max == INFINITE and self.goalBias and (random.random() < GOAL_BIAS or goal_bias_success):
                # GOAL BIAS
                preRandomPt = None
                print("Goal Bias! @ {}".format(self.stats.valid_sample))
                # for goal bias, we pick a point that is not blocked
                # nn = None
                nn = None
                for p in self.nodes:
                    if checkIntersect(p, self.goalPt, self.img):
                        if nn is None:
                            goal_bias_success = True
                            newnode = self.goalPt
                            nn = p
                        if dist(p.pos, self.goalPt.pos) < dist(nn.pos, self.goalPt.pos):
                            nn = p

            if not goal_bias_success:
                rand = Node(self.sampler.getNextNode())
                preRandomPt = rand
                nn = self.nodes[0]
                # for Non-goal bias, we pick the cloest point
                for p in self.nodes:
                    if dist(p.pos, rand.pos) < dist(nn.pos, rand.pos):
                        nn = p

                interpolatedPoint = self.step_from_to(nn.pos, rand.pos)
                newnode = Node(interpolatedPoint)

            if self.check_entire_path:
                ## CHECK entire PATH
                checkingNode = rand
            else:
                ## CHECK only INTERMEDIATE path
                checkingNode = newnode
            if not checkIntersect(nn, checkingNode, self.img):
                self.sampler.addSample(p=preRandomPt, free=False)
                # self.sampler.addSample(p=preRandomPt, free=True, weight=10)
                self.stats.addInvalid(perm=False)
            else:
                self.stats.addFree()
                x = newnode.pos[0]
                y = newnode.pos[1]
                try:
                    self.sampler.addTreeNode(x, y)
                except AttributeError:
                    pass
                if preRandomPt is not None:
                    # add all in between point of nearest node of the random pt as valid
                    try:
                        x1 = preRandomPt.pos[0]
                        y1 = preRandomPt.pos[1]

                        if not self.check_entire_path:
                            (_, _), (x1, x2) = getCoorBeforeCollision(nn, rand, self.img)

                        self.sampler.addSampleLine(x, y, x1, y1)
                    except AttributeError:
                        pass
                else:
                    if checkIntersect(nn, rand, self.img):
                        # Reaching this point means the goal bias had been successful. Go directly to the goal!
                        goal_bias_success = True
                #######################
                [newnode, nn] = self.chooseParent(nn, newnode)

                self.nodes.append(newnode)
                pygame.draw.line(self.path_layers, black, nn.pos*SCALING, newnode.pos*SCALING, SCALING)
                self.reWire(newnode)
                pygame.display.update()

                if dist(newnode.pos, self.goalPt.pos) < GOAL_RADIUS:
                    # print('Reached goal!')

                    if newnode.cost < self.c_max:
                        self.c_max = newnode.cost
                        self.drawSolutionPath()

                for e in pygame.event.get():
                    if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving.")
            self.update_screen(self.stats.valid_sample)

        self.wait_for_exit()

    def wait_for_exit(self):
        # self.update_screen()
        # wait for exit
        while True:
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")


    def update_screen(self, counter):
        # limites the screen update
        # if counter % 100 == 0:
        if counter % 10 == 0:
            # background
            self.window.blit(self.background,(0,0))
            try:
                self.sampler.paint(self.window)
            except AttributeError:
                pass

        if counter % 5 == 0:
            self.window.blit(self.path_layers,(0,0))
            self.window.blit(self.solution_path_screen,(0,0))

            if self.startPt is not None:
                pygame.draw.circle(self.path_layers, red, self.startPt.pos*SCALING, GOAL_RADIUS*SCALING)
            if self.goalPt is not None:
                pygame.draw.circle(self.path_layers, blue, self.goalPt.pos*SCALING, GOAL_RADIUS*SCALING)


        if counter % 2 == 0:
            _cost = 'INF' if self.c_max == INFINITE else round(self.c_max, 2)
            text = 'Cost_min: {}  | Nodes: {}'.format(_cost, counter)
            self.window.blit(self.myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,self.YDIM * SCALING * 0.88))
            text = 'Invalid sample: {}(temp) {}(perm)'.format(self.stats.invalid_sample_temp, self.stats.invalid_sample_perm)
            self.window.blit(self.myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,self.YDIM * SCALING * 0.95))

        pygame.display.update()
