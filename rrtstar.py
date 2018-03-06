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

GOAL_RADIUS = 10

SCALING = 3

img = None
c_min = None
x_center = None
angle = None

c_max = sys.maxsize
foundSolution = False


############################################################

def collides(p):    #check if point is white (which means free space)
    global pygame, img
    # make sure x and y is within image boundary
    x = int(p[0])
    y = int(p[1])
    if x < 0 or x >= img.get_width() or y < 0 or y >= img.get_height():
        # print(x, y)
        return True
    color = img.get_at((x, y))
    white = 255, 255, 255
    # print(color)
    if color == pygame.Color(*white):
        return False
    return True


def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


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
            pygame.draw.line(screen, white, p.pos, p.parent.pos)
            p.parent = newnode
            p.cost = newnode.cost + dist(p.pos, newnode.pos)
            nodes[i] = p
            pygame.draw.line(screen, black, p.pos, newnode.pos)
    return nodes

# to force drawSolutionPath only draw once for every new solution
solution_path_c_max = sys.maxsize
solution_path_color = None


def drawSolutionPath(start, goal, nodes, pygame, screen):
    if not foundSolution:
        return
    global solution_path_c_max, solution_path_color, c_max
    if solution_path_c_max > c_max:
        # new color for new path
        solution_path_c_max = c_max
        solution_path_color = random.random()*255, random.random()*255, random.random()*255
        print('Cost: {}'.format(c_max))

    # redraw the old path
    nn = nodes[0]
    for p in nodes:
        if dist(p.pos, goal.pos) < dist(nn.pos, goal.pos):
            nn = p
    while nn != start:
        pygame.draw.line(screen, solution_path_color, nn.pos, nn.parent.pos, 5)
        nn = nn.parent



def get_random_path(c_max):
    if c_max != sys.maxsize: #max size represent infinite (not found solution yet)
        global c_min, x_center, angle
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
        return x2 + x_center[0] , y2 + x_center[1]

    # Random path
    while True:
        p = random.random()*XDIM, random.random()*YDIM
        if not collides(p):
            return p

class Node:
    pos = None  # index 0 is x, index 1 is y
    cost = 0
    parent = None

    def __init__(self, pos):
        self.pos = pos

def drawEllipse():
    pass

def main():
    global pygame, img

    # initialize and prepare screen
    # a=checkIntersect()
    # print(a)
    pygame.init()
    img = pygame.image.load('map.png')
    XDIM = img.get_width()
    YDIM = img.get_height()

    screen = pygame.Surface( [XDIM, YDIM] )
    window = pygame.display.set_mode([XDIM * SCALING, YDIM * SCALING])
    # solution_path_screen = pygame.Surface( [XDIM, YDIM], pygame.SRCALPHA, 32 )
    # solution_path_screen = solution_path_screen.convert_alpha()
    solution_path_screen = pygame.Surface( [XDIM, YDIM] )
    ck = 255,0,255
    solution_path_screen.fill(ck)
    # solution_path_screen.set_colorkey(ck)
    # solution_path_screen.set_alpha(255)


    def update():
        pygame.transform.scale(screen, (XDIM * SCALING, YDIM * SCALING), window)
        # pygame.transform.scale(solution_path_screen, (XDIM * SCALING, YDIM * SCALING), window)
        pygame.display.update()
        # screen.blit()

    pygame.display.set_caption('RRTstar')
    white = 255, 255, 255
    black = 20, 20, 40
    red = 255, 0, 0
    green = 0, 255, 0
    blue = 0, 0, 255
    # screen.fill(white)
    window.blit(solution_path_screen,(0,0))
    screen.blit(img,(0,0))


    nodes = []

    ##################################################
    # Get starting and ending point
    startPt = None
    goalPt = None
    print('Select Starting Point and then Goal Point')
    fpsClock.tick(10)
    while startPt is None or goalPt is None:
        for e in pygame.event.get():
            if e.type == MOUSEBUTTONDOWN:
                mousePos = (int(e.pos[0] / SCALING), int(e.pos[1] / SCALING))
                if startPt is None:
                    if collides(mousePos) == False:
                        print(('starting point set: ' + str(mousePos)))
                        startPt = Node(mousePos)
                        nodes.append(startPt)
                        pygame.draw.circle(
                            screen, red, startPt.pos, GOAL_RADIUS)
                elif goalPt is None:
                    if collides(mousePos) == False:
                        print(('goal point set: ' + str(mousePos)))
                        goalPt = Node(mousePos)
                        pygame.draw.circle(
                            screen, blue, goalPt.pos, GOAL_RADIUS)
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
    global foundSolution, c_max

    for i in range(NUMNODES):
        rand = Node(get_random_path(c_max))
        nn = nodes[0]
        for p in nodes:
            if dist(p.pos, rand.pos) < dist(nn.pos, rand.pos):
                nn = p
        interpolatedNode = step_from_to(nn.pos, rand.pos)

        newnode = Node(interpolatedNode)
        if checkIntersect(nn, rand, img):

            [newnode, nn] = chooseParent(nn, newnode, nodes)
            # newnode.parent = nn

            nodes.append(newnode)
            pygame.draw.line(screen, black, nn.pos, newnode.pos)
            nodes = reWire(nodes, newnode, pygame, screen)
            pygame.display.update()

            if dist(newnode.pos, goalPt.pos) < GOAL_RADIUS:
                print('Reached goal!')
                # goalPt.parent = newnode
                # drawSolutionPath(startPt, goalPt, nodes, pygame, screen)
                # print(newnode.cost)
                # print(c_min)
                # print(x_center)
                # print(angle)

                foundSolution = True
                if newnode.cost < c_max:
                    c_max = newnode.cost

                # drawSolutionPath(startPt, goalPt, nodes, pygame, screen)
                # drawEllipse()
                #
                # # get_random_path(newnode.cost, screen=screen)
                #
                # update(screen, window)
                # import time
                # time.sleep(100)
                # break

            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving.")
        drawSolutionPath(startPt, goalPt, nodes, pygame, solution_path_screen)
        update()
    update()
    exit()



# if python says run, then we should run
if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
