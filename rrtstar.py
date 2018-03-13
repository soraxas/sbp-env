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
from checkCollision import *
import numpy as np

# constants
XDIM = 640
YDIM = 480
EPSILON = 7.0
NUMNODES = 2000
RADIUS = 15
fpsClock = pygame.time.Clock()

INFINITE = sys.maxsize

ALPHA_CK = 255,0,255

GOAL_RADIUS = 10

SCALING = 2

GOAL_BIAS = 0.05

img = None
c_min = None
x_center = None
angle = None
c_max = INFINITE

likelihoods = None

invalid_sample = 0
invalid_sample_wall = 0

class prob_block:
    def __init__(self, x, y, width, height, alpha):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.alpha = alpha

############################################################

import kde
counter = 0
kernel = None
kernel_pts = None
kernel_perma_pts = None

from matplotlib import pyplot as plt
COUNTCOUNT = 0
prob_vector = None
prob_vector_normalized = None

prob_vector_locks = None
PROB_BLOCK_SIZE = 10
def addInvalidPoint(p, blockedSpace, perma=False):
    global COUNTCOUNT
    if p is None:
        return
    try:
        p = p.pos
    except AttributeError as e:
        pass
    global prob_vector, prob_vector_locks, prob_vector_normalized


    if True:
        x = int(p[0]/PROB_BLOCK_SIZE)
        y = int(p[1]/PROB_BLOCK_SIZE)
        if x < 0 or x >= prob_vector.shape[0] or \
            y < 0 or y >= prob_vector.shape[1]:
                return
        # # print('{},{}'.format(x,y))
        factor = 1.5
        # max_allow = (1 / prob_vector.size) * 2 # max prob allow to prevent over concentration
        # min_allow = 1 / prob_vector.size / 2
        if not prob_vector_locks[x][y]:
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
                        prob_vector[x][y] -= (100-prob_vector[x][y])*0.2
                        if prob_vector[x][y] < 15:
                            prob_vector[x][y] = 15
                # else:
                    # pass
                    # prob_vector[x][y] = 0
                    # prob_vector_locks[x][y] = 1

            else:
                    pass
                    if prob_vector[x][y] < 100:
                        # prob_vector[x][y] += 1
                        prob_vector[x][y] += (100-prob_vector[x][y])*0.5
                        if prob_vector[x][y] > 100:
                            prob_vector[x][y] = 100
                        # prob_vector[x][y] = 100
            #     if prob_vector[x][y] > min_allow:
                    # prob_vector[x][y] *= factor
            #     # prob_vector[x][y] =0
                    # prob_vector[x][y] = 1



        # else:
        #     if prob_vector[x][y] > min_allow:
        #         prob_vector[x][y] /= factor

#########################################################
# smooth the prob 2d vector
        import scipy as sp
        import scipy.ndimage
        sigma_y = 1.0
        sigma_x = 1.0
        sigma = [sigma_y, sigma_x]
        if COUNTCOUNT % 200 == 0 or True:
            pass
            # prob_vector_normalized = sp.ndimage.filters.gaussian_filter(prob_vector, sigma, mode='reflect')
            prob_vector_normalized = np.copy(prob_vector)
            prob_vector_normalized /= prob_vector_normalized.sum()
        COUNTCOUNT += 1
        # import time
        # time.sleep(2)
#########################################################


############################################################

def collides(p):    #check if point is white (which means free space)
    global pygame, img, invalid_sample_wall
    # make sure x and y is within image boundary
    x = int(p[0])
    y = int(p[1])
    if x < 0 or x >= img.get_width() or y < 0 or y >= img.get_height():
        # print(x, y)
        # addInvalidPoint(p, True, perma=True)
        return True
    color = img.get_at((x, y))
    white = 255, 255, 255
    # print(color)
    if color == pygame.Color(*white):
        return False
    addInvalidPoint(p, True, perma=True)
    invalid_sample_wall += 1
    return True


def dist(p1, p2):
    return np.linalg.norm(p1 - p2)
    # return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return np.array(p2)
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        pos = p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)
        return np.array(pos)


def chooseParent(nn, newnode, nodes):
    global img
    for p in nodes:
        if checkIntersect(p, newnode, img) and dist(p.pos, newnode.pos) < RADIUS and p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos):
            nn = p
    newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
    newnode.parent = nn
    return newnode, nn


def reWire(nodes, newnode, pygame, screen):
    global img
    white = 255, 240, 200
    black = 20, 20, 40
    for i in range(len(nodes)):
        p = nodes[i]
        if checkIntersect(p, newnode, img) and p != newnode.parent and dist(p.pos, newnode.pos) < RADIUS and newnode.cost + dist(p.pos, newnode.pos) < p.cost:
            pygame.draw.line(screen, white, p.pos*SCALING, p.parent.pos*SCALING, SCALING)
            p.parent = newnode
            p.cost = newnode.cost + dist(p.pos, newnode.pos)
            nodes[i] = p
            pygame.draw.line(screen, black, p.pos*SCALING, newnode.pos*SCALING, SCALING)
    return nodes

# to force drawSolutionPath only draw once for every new solution
solution_path_c_max = INFINITE

def drawSolutionPath(start, goal, nodes, pygame, screen):
    global solution_path_c_max, c_max

    # redraw new path
    green = 0,150,0
    screen.fill(ALPHA_CK)

    nn = nodes[0]
    for p in nodes:
        if dist(p.pos, goal.pos) < dist(nn.pos, goal.pos):
            nn = p
    while nn != start:
        pygame.draw.line(screen, green, nn.pos*SCALING, nn.parent.pos*SCALING, 5*SCALING)
        nn = nn.parent



def get_random_path(c_max):
    if c_max != INFINITE: #max size represent infinite (not found solution yet)
        while True:
            global c_min, x_center, angle, prob_vector_normalized
            # already have a valid solution, optimise in ellipse region
            r1 = c_max / 2
            r2 = math.sqrt(abs(c_max**2 - c_min**2))

            x = np.random.uniform(-1, 1)
            y = np.random.uniform(-1, 1)

            x2 =  x * r1 * math.cos(angle) + y * r2 * math.sin(angle)
            y2 = -x * r1 * math.sin(angle) + y * r2 * math.cos(angle)

            ##################################
            ##################################
            ##################################
            pos =  x2 + x_center[0] , y2 + x_center[1]
            if not collides(pos):
                return np.array(pos)

    # Random path
    # prob_vector_normalized = prob_vector
    if prob_vector_normalized is None:
    # if prob_vector_normalized is None:
        while True:
            p = random.random()*XDIM,  random.random()*YDIM
            # p = random.random()*XDIM, random.random()*YDIM
            if not collides(p):
                return np.array(p)
    else:
        # np.random.choice()
        # print(prob_vector_normalized.size)
        # print(prob_vector_normalized.shape)
        while True:
            choice = np.random.choice(range(prob_vector_normalized.size), p=prob_vector_normalized.ravel())
            y = choice % prob_vector_normalized.shape[1]
            x = int(choice / prob_vector_normalized.shape[1])
            print('{},{}'.format(x,y))
            print(prob_vector_normalized)

            p = (x + random.random())*PROB_BLOCK_SIZE, (y + random.random())*PROB_BLOCK_SIZE
            print(p)
            # p = random.random()*XDIM, random.random()*YDIM
            if not collides(p):
                return np.array(p)

class Node:
    pos = None  # index 0 is x, index 1 is y
    cost = 0
    parent = None

    def __init__(self, pos):
        self.pos = pos



def main():
    global pygame, img, likelihoods, prob_vector_normalized, prob_vector, prob_vector_locks, prob_vector_normalized

    # initialize and prepare screen
    pygame.init()
    img = pygame.image.load('map.png')
    XDIM = img.get_width()
    YDIM = img.get_height()

    pygame.display.set_caption('RRTstar')
    white = 255, 255, 255
    black = 20, 20, 40
    red = 255, 0, 0
    blue = 0, 0, 255
    # screen.fill(white)
    ################################################################################
    # text
    pygame.font.init()
    myfont = pygame.font.SysFont('Arial', 15 * SCALING)
    ################################################################################
    # main window
    window = pygame.display.set_mode([XDIM * SCALING, YDIM * SCALING])
    ################################################################################
    # probability layer
    prob_layer = pygame.Surface((PROB_BLOCK_SIZE * SCALING,PROB_BLOCK_SIZE * SCALING), pygame.SRCALPHA)
    ################################################################################
    # background aka the room
    background = pygame.Surface( [XDIM, YDIM] )
    background.blit(img,(0,0))
    # resize background to match windows
    background = pygame.transform.scale(background, [XDIM * SCALING, YDIM * SCALING])
    ################################################################################
    # path of RRT*
    path_layers = pygame.Surface( [XDIM * SCALING, YDIM * SCALING] )
    path_layers.fill(ALPHA_CK)
    path_layers.set_colorkey(ALPHA_CK)
    # rescale to make it bigger
    path_layers_rescale = pygame.Surface( [XDIM * SCALING, YDIM * SCALING] )
    path_layers_rescale.fill(ALPHA_CK)
    path_layers_rescale.set_colorkey(ALPHA_CK)
    ################################################################################
    # layers to store the solution path
    solution_path_screen = pygame.Surface( [XDIM * SCALING, YDIM * SCALING] )
    solution_path_screen.fill(ALPHA_CK)
    solution_path_screen.set_colorkey(ALPHA_CK)
    # rescale to make it bigger
    solution_path_screen_rescale = pygame.Surface( [XDIM * SCALING, YDIM * SCALING] )
    solution_path_screen_rescale.fill(ALPHA_CK)
    solution_path_screen_rescale.set_colorkey(ALPHA_CK)
    ################################################################################

    startPt = None
    goalPt = None

    num_nodes = 0

    if prob_vector is None:
        shape = (int(XDIM/PROB_BLOCK_SIZE) + 1, int(YDIM/PROB_BLOCK_SIZE) + 1 )
        prob_vector = np.ones(shape)
        prob_vector *= 20
        prob_vector_locks = np.zeros(shape)
        # prob_vector *= 10

        # prob_vector /= prob_vector.sum()

    def update():
        # pygame.transform.scale(path_layers, (XDIM * SCALING, YDIM * SCALING), path_layers_rescale)
        # pygame.transform.scale(solution_path_screen, (XDIM * SCALING, YDIM * SCALING), solution_path_screen_rescale)

        # limites the screen update
        # if num_nodes % 100 == 0:
        if num_nodes % 10 == 0:
            # background
            window.blit(background,(0,0))
            # prob likelihoods
            # prob_vector_normalized = prob_vector
            if prob_vector_normalized is not None:
                max_prob = prob_vector_normalized.max()
                min_prob = prob_vector_normalized.min()
                for i in range(prob_vector_normalized.shape[0]):
                    for j in range(prob_vector_normalized.shape[1]):
                        a = max_prob-min_prob
                        if a == 0:
                            a = 1
                        alpha = 240 * (1 -(prob_vector_normalized[i][j]-min_prob)/a)
                        if not prob_vector_locks[i][j]:
                            prob_layer.fill((255,128,255,alpha))
                        else:
                            prob_layer.fill((0,255,0,alpha))
                        # print(prob_vector_normalized[i][j])
                        window.blit(prob_layer, (i*PROB_BLOCK_SIZE*SCALING,j*PROB_BLOCK_SIZE*SCALING))

        if num_nodes % 5 == 0:
            window.blit(path_layers,(0,0))
            window.blit(solution_path_screen,(0,0))

            if startPt is not None:
                pygame.draw.circle(path_layers, red, startPt.pos*SCALING, GOAL_RADIUS*SCALING)
            if goalPt is not None:
                pygame.draw.circle(path_layers, blue, goalPt.pos*SCALING, GOAL_RADIUS*SCALING)



        if num_nodes % 2 == 0:
            _cost = 'INF' if c_max == INFINITE else round(c_max, 2)
            text = 'Cost_min: {}  | Nodes: {}  |  Invalid sample: {}(temp) {}(perm)'.format(_cost, num_nodes, invalid_sample, invalid_sample_wall)
            window.blit(myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,YDIM * SCALING * 0.9))

            if kernel_pts is not None and kernel_perma_pts is not None:
                text = 'kernel_pt:  {}  |  kernel_pt_perma:  {}  '.format(len(kernel_pts[0]), len(kernel_perma_pts[0]))
                window.blit(myfont.render(text, False, (0, 0, 0), (255,255,255)), (20,YDIM * SCALING * 0.95))

            pygame.display.update()

    nodes = []
    ##################################################
    # Get starting and ending point
    print('Select Starting Point and then Goal Point')
    fpsClock.tick(10)
    while startPt is None or goalPt is None:
        for e in pygame.event.get():
            if e.type == MOUSEBUTTONDOWN:
                mousePos = (int(e.pos[0] / SCALING), int(e.pos[1] / SCALING))
                if startPt is None:
                    if collides(mousePos) == False:
                        print(('starting point set: ' + str(mousePos)))
                        startPt = Node(np.array(mousePos))
                        nodes.append(startPt)
                elif goalPt is None:
                    if collides(mousePos) == False:
                        print(('goal point set: ' + str(mousePos)))
                        goalPt = Node(np.array(mousePos))
                elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")
        update()
    ##################################################
    # calculate information regarding shortest path
    global c_min, x_center, angle
    c_min = dist(startPt.pos, goalPt.pos)
    x_center = (startPt.pos[0]+goalPt.pos[0])/2 , (startPt.pos[1]+goalPt.pos[1])/2
    dy = goalPt.pos[1] - startPt.pos[1]
    dx = goalPt.pos[0] - startPt.pos[0]
    angle = math.atan2(-dy, dx)

    ##################################################

    fpsClock.tick(10000)
    global c_max, invalid_sample,invalid_sample_wall

    for i in range(NUMNODES):
        # probabiilty to bias toward goal (while not reaching goal yet)
        if c_max == INFINITE and random.random() < GOAL_BIAS:
            rand = Node(np.array(goalPt.pos))
            preRandomPt = None
        else:
            rand = Node(get_random_path(c_max))
            preRandomPt = rand
        nn = nodes[0]
        for p in nodes:
            if dist(p.pos, rand.pos) < dist(nn.pos, rand.pos):
                nn = p
        interpolatedNode = step_from_to(nn.pos, rand.pos)

        newnode = Node(interpolatedNode)
        if not checkIntersect(nn, rand, img):
            addInvalidPoint(preRandomPt, True)
            invalid_sample += 1
        else:
            x = int(interpolatedNode[0] / PROB_BLOCK_SIZE)
            y = int(interpolatedNode[1] / PROB_BLOCK_SIZE)
            prob_vector[x][y] = 10
            prob_vector_locks[x][y] = 1
            addInvalidPoint(preRandomPt, False)
            [newnode, nn] = chooseParent(nn, newnode, nodes)
            # newnode.parent = nn

            nodes.append(newnode)
            pygame.draw.line(path_layers, black, nn.pos*SCALING, newnode.pos*SCALING, SCALING)
            nodes = reWire(nodes, newnode, pygame, path_layers)
            pygame.display.update()

            if dist(newnode.pos, goalPt.pos) < GOAL_RADIUS:
                # print('Reached goal!')

                if newnode.cost < c_max:
                    c_max = newnode.cost
                    drawSolutionPath(startPt, goalPt, nodes, pygame, solution_path_screen)

            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")
        num_nodes = i
        update()
    update() # update one last time
    # wait for exit
    while True:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving.")


# if python says run, then we should run
if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
