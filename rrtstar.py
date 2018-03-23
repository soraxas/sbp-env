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
# import rrt_policy


# constants

INFINITE = sys.maxsize

ALPHA_CK = 255,0,255

GOAL_RADIUS = 10

SCALING = 2

GOAL_BIAS = 0.01

distinguishLockedBlock = True

##################################################
## COLOR ##
white = 255, 255, 255
black = 20, 20, 40
red = 255, 0, 0
blue = 0, 0, 255
green = 0,150,0
##################################################


class prob_block:
    def __init__(self, x, y, width, height, alpha):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.alpha = alpha

class Node:
    pos = None  # index 0 is x, index 1 is y
    cost = 0
    parent = None

    def __init__(self, pos):
        self.pos = pos

############################################################

from matplotlib import pyplot as plt

class RRT:

    def __init__(self, image, epsilon, max_number_nodes, radius, prob_block_size):
        # initialize and prepare screen
        pygame.init()
        self.img = pygame.image.load(image)
        self.XDIM = self.img.get_width()
        self.YDIM = self.img.get_height()

        self.EPSILON = epsilon
        self.NUMNODES = max_number_nodes
        self.RADIUS = radius
        self.fpsClock = pygame.time.Clock()

        self.c_max = INFINITE

        self.invalid_sample = 0
        self.invalid_sample_perma = 0

        self.num_invalid_pt = 0

        self.PROB_BLOCK_SIZE = prob_block_size

        shape = (int(self.XDIM/self.PROB_BLOCK_SIZE) + 1, int(self.YDIM/self.PROB_BLOCK_SIZE) + 1 )
        self.prob_vector = np.ones(shape)
        self.prob_vector *= 20
        self.prob_vector_normalized = None
        self.prob_vector_locks = np.zeros(shape)

        self.tree_vector = np.ones(shape)


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
        # probability layer
        self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * SCALING,self.PROB_BLOCK_SIZE * SCALING), pygame.SRCALPHA)
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
                        if self.collides(mousePos) == False:
                            print(('starting point set: ' + str(mousePos)))
                            self.startPt = Node(np.array(mousePos))
                            self.nodes.append(self.startPt)

                    elif self.goalPt is None:
                        if self.collides(mousePos) == False:
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


    ############################################################

    def collides(self, p):    #check if point is white (which means free space)
        # make sure x and y is within image boundary
        x = int(p[0])
        y = int(p[1])
        if x < 0 or x >= self.img.get_width() or y < 0 or y >= self.img.get_height():
            return True
        color = self.img.get_at((x, y))
        # print(color)
        if color == pygame.Color(*white):
            return False
        self.addInvalidPoint(p, True, perma=True)
        self.invalid_sample_perma += 1
        return True


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



    def addInvalidPoint(self,p, blockedSpace, perma=False, alreadyDividedByProbBlockSize=False):
        if p is None:
            return
        try:
            p = p.pos
        except AttributeError as e:
            pass

        if True:
            if not alreadyDividedByProbBlockSize:
                x = int(p[0]/self.PROB_BLOCK_SIZE)
                y = int(p[1]/self.PROB_BLOCK_SIZE)
            else:
                x = p[0]
                y = p[1]
            if x < 0 or x >= self.prob_vector.shape[0] or \
                y < 0 or y >= self.prob_vector.shape[1]:
                    return
            # # print('{},{}'.format(x,y))
            factor = 1.5
            # max_allow = (1 / prob_vector.size) * 2 # max prob allow to prevent over concentration
            # min_allow = 1 / prob_vector.size / 2
            if not self.prob_vector_locks[x][y]:
                if blockedSpace:
                #     # prob_vector[x][y] = 1
                #     if prob_vector[x][y] < max_allow:
                #         # prob_vector[x][y] = max_allow
                        # prob_vector[x][y] /= factor
                        # prob_vector[x][y] = 0
                #         print(prob_vector[x][y])
                    # pass
                    # if not perma:
                        # if prob_vector[x][y] > :
                            # prob_vector[x][y] -= 1
                            self.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.2
                            if self.prob_vector[x][y] < 15:
                                self.prob_vector[x][y] = 15
                    # else:
                        # pass
                        # prob_vector[x][y] = 0
                        # prob_vector_locks[x][y] = 1

                else:
                    # if False:
                    # fix it at 75%
                    self.prob_vector[x][y] = 100
                    # self.prob_vector_locks[x][y] = 1
                    # pass
                    if False:
                        # if self.prob_vector[x][y] < 100:
                            # prob_vector[x][y] += 1
                            self.prob_vector[x][y] += (100-self.prob_vector[x][y])*0.5
                            if self.prob_vector[x][y] > 100:
                                self.prob_vector[x][y] = 100
                            # prob_vector[x][y] = 100
                #     if prob_vector[x][y] > min_allow:
                        # prob_vector[x][y] *= factor
                #     # prob_vector[x][y] =0
                        # prob_vector[x][y] = 1



            # else:
            #     if prob_vector[x][y] > min_allow:
            #       distinguishLockedBlock and   prob_vector[x][y] /= factor

    #########################################################
    # smooth the prob 2d vector
            import scipy as sp
            import scipy.ndimage
            sigma_y = 1.0
            sigma_x = 1.0
            sigma = [sigma_y, sigma_x]
            if self.num_invalid_pt % 200 == 0 or True:
                pass
                self.prob_vector_normalized = np.copy(self.prob_vector)
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(self.prob_vector_normalized, sigma, mode='reflect')
                self.prob_vector_normalized *= 1/self.tree_vector
                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            self.num_invalid_pt += 1
            # import time
            # time.sleep(2)
    #########################################################

    def get_random_path(self):
        if self.c_max != INFINITE: #max size represent infinite (not found solution yet)
            while True:
                # already have a valid solution, optimise in ellipse region
                r1 = self.c_max / 2
                r2 = math.sqrt(abs(self.c_max**2 - self.c_min**2))

                x = np.random.uniform(-1, 1)
                y = np.random.uniform(-1, 1)

                x2 =  x * r1 * math.cos(self.angle) + y * r2 * math.sin(self.angle)
                y2 = -x * r1 * math.sin(self.angle) + y * r2 * math.cos(self.angle)

                ##################################
                ##################################
                ##################################
                pos =  x2 + self.x_center[0] , y2 + self.x_center[1]
                if not self.collides(pos):
                    return np.array(pos)

        # Random path
        # self.prob_vector_normalized = prob_vector
        if self.prob_vector_normalized is None:
        # if self.prob_vector_normalized is None:
            while True:
                p = random.random()*self.XDIM,  random.random()*self.YDIM
                # p = random.random()*self.XDIM, random.random()*self.YDIM
                if not self.collides(p):
                    return np.array(p)
        else:
            # np.random.choice()
            # print(self.prob_vector_normalized.size)
            # print(self.prob_vector_normalized.shape)
            while True:
                choice = np.random.choice(range(self.prob_vector_normalized.size), p=self.prob_vector_normalized.ravel())
                y = choice % self.prob_vector_normalized.shape[1]
                x = int(choice / self.prob_vector_normalized.shape[1])
                # print('{},{}'.format(x,y))
                # print(self.prob_vector_normalized)

                p = (x + random.random())*self.PROB_BLOCK_SIZE, (y + random.random())*self.PROB_BLOCK_SIZE
                # print(p)
                # p = random.random()*self.XDIM, random.random()*self.YDIM
                if not self.collides(p):
                    return np.array(p)








    def run(self):

        ##################################################

        self.fpsClock.tick(10000)
        goal_bias_success = False
        for i in range(self.NUMNODES):
            # probabiilty to bias toward goal (while not reaching goal yet)
            if self.c_max != INFINITE:
                self.wait_for_exit()
                pass

            if self.c_max == INFINITE and (random.random() < GOAL_BIAS or goal_bias_success):
                rand = Node(np.array(self.goalPt.pos))
                preRandomPt = None
                print("Goal Bias! @ {}".format(i))
            else:
                rand = Node(self.get_random_path())
                preRandomPt = rand

            nn = self.nodes[0]
            if preRandomPt is not None:
                # for Non-goal bias, we pick the cloest point
                for p in self.nodes:
                    if dist(p.pos, rand.pos) < dist(nn.pos, rand.pos):
                        nn = p
            else:
                # for goal bias, we pick a point that is not blocked
                nn = None
                for p in self.nodes:
                    if checkIntersect(p, rand, self.img):
                        nn = p
                        break
                if nn is not None:
                    for p in self.nodes:
                        if checkIntersect(p, rand, self.img) and ((dist(p.pos, rand.pos) < dist(nn.pos, rand.pos))):
                            nn = p
                else:
                    # cannot find, use base case
                    nn = self.nodes[0]

            interpolatedPoint = self.step_from_to(nn.pos, rand.pos)
            newnode = Node(interpolatedPoint)
            if not checkIntersect(nn, rand, self.img):
                self.addInvalidPoint(preRandomPt, True)
                self.invalid_sample += 1
            else:
                x = int(interpolatedPoint[0] / self.PROB_BLOCK_SIZE)
                y = int(interpolatedPoint[1] / self.PROB_BLOCK_SIZE)
                # self.prob_vector[x][y] = 30
                # self.prob_vector_locks[x][y] = 1

                self.tree_vector[x][y] += 1
                if preRandomPt is not None:
                    # add all in between point of nearest node of the random pt as valid
                    x1 = int(preRandomPt.pos[0] / self.PROB_BLOCK_SIZE)
                    y1 = int(preRandomPt.pos[1] / self.PROB_BLOCK_SIZE)
                    points = get_line((x,y), (x1,y1))
                    # print(points)
                    for p in points:
                        self.addInvalidPoint(p, False, alreadyDividedByProbBlockSize=True)
                else:
                    # Reaching this point means the goal bias had been successful. Go directly to the goal!
                    goal_bias_success = True
                #######################
                [newnode, nn] = self.chooseParent(nn, newnode)
                # newnode.parent = nn

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
            self.update_screen(i)

        self.wait_for_exit()

    def wait_for_exit(self):
        self.update_screen()
        # wait for exit
        while True:
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")

    def get_vector_alpha_parameters(self, vector):
        max_prob = self.prob_vector_normalized.max()
        min_prob = self.prob_vector_normalized.min()
        denominator = max_prob-min_prob
        if denominator == 0:
            denominator = 1 # prevent division by zero
        return max_prob, min_prob, denominator


    def update_screen(self, counter=-10):
        # limites the screen update
        # if counter % 100 == 0:
        if counter % 10 == 0:
            # background
            self.window.blit(self.background,(0,0))
            # prob likelihoods
            # self.prob_vector_normalized = prob_vector
            if self.prob_vector_normalized is not None:
                for i in range(self.prob_vector_normalized.shape[0]):
                    for j in range(self.prob_vector_normalized.shape[1]):
                        max_max_prob, min_prob, denominator = self.get_vector_alpha_parameters(self.prob_vector_normalized)
                        alpha = 240 * (1 -(self.prob_vector_normalized[i][j]-min_prob)/denominator)

                        distinguishLockedBlock = False
                        if distinguishLockedBlock and self.tree_vector[i][j] > 1:
                            max_max_prob, min_prob, denominator = self.get_vector_alpha_parameters(self.tree_vector)
                            alpha = 240 * (1 - (self.prob_vector_normalized[i][j]-min_prob)/denominator)
                            self.prob_layer.fill((0,255,0,alpha))
                        else:
                            self.prob_layer.fill((255,128,255,alpha))
                        # print(self.prob_vector_normalized[i][j])
                        self.window.blit(self.prob_layer, (i*self.PROB_BLOCK_SIZE*SCALING,j*self.PROB_BLOCK_SIZE*SCALING))

        if counter % 5 == 0:
            self.window.blit(self.path_layers,(0,0))
            self.window.blit(self.solution_path_screen,(0,0))

            if self.startPt is not None:
                pygame.draw.circle(self.path_layers, red, self.startPt.pos*SCALING, GOAL_RADIUS*SCALING)
            if self.goalPt is not None:
                pygame.draw.circle(self.path_layers, blue, self.goalPt.pos*SCALING, GOAL_RADIUS*SCALING)


        if counter % 2 == 0 and counter >= 0:
            _cost = 'INF' if self.c_max == INFINITE else round(self.c_max, 2)
            text = 'Cost_min: {}  | Nodes: {}  |  Invalid sample: {}(temp) {}(perm)'.format(_cost, counter, self.invalid_sample, self.invalid_sample_perma)
            self.window.blit(self.myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,self.YDIM * SCALING * 0.9))

            # if kernel_pts is not None and kernel_perma_pts is not None:
            #     text = 'kernel_pt:  {}  |  kernel_pt_perma:  {}  '.format(len(kernel_pts[0]), len(kernel_perma_pts[0]))
            #     self.window.blit(self.myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,self.YDIM * SCALING * 0.95))

        pygame.display.update()


# if python says run, then we should run
if __name__ == '__main__':
    epsilon = 7.0
    max_number_nodes = 2000
    radius = 15
    rrt = RRT(image='map.png', epsilon=epsilon, max_number_nodes=max_number_nodes, radius=radius, prob_block_size=8)
    rrt.run()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
