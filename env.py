#!/usr/bin/env python
import logging
import sys
from math import atan2, cos, sin

import pygame
from pygame.locals import *

from collisionChecker import *
from helpers import Node, check_pygame_enabled, MagicDict, Stats

LOGGER = logging.getLogger(__name__)

############################################################


class Env:
    def __init__(self,
                 startPt=None,
                 goalPt=None,
                 **kwargs):
        # initialize and prepare screen
        self.args = MagicDict(kwargs)
        self.img = pygame.image.load(self.args.image)
        self.cc = CollisionChecker(self.img)
        self.XDIM = self.img.get_width()
        self.YDIM = self.img.get_height()
        self.extra = 25
        self.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        self.planner = self.args.planner
        self.planner.args.env = self

        self.pygame_init(kwargs['enable_pygame'])

        ##################################################
        # Get starting and ending point
        LOGGER.info('Select Starting Point and then Goal Point')
        self.startPt = None
        self.goalPt = None
        while self.startPt is None or self.goalPt is None:
            if self.args.enable_pygame:
                for e in pygame.event.get():
                    if e.type == MOUSEBUTTONDOWN:
                        mousePos = (int(e.pos[0] / self.args.scaling),
                                    int(e.pos[1] / self.args.scaling))
                        if startPt is None:
                            if not self.cc.collides(mousePos):
                                LOGGER.info(
                                    ('starting point set: ' + str(mousePos)))
                                startPt = mousePos
                        elif goalPt is None:
                            if not self.cc.collides(mousePos):
                                LOGGER.info(('goal point set: ' + str(mousePos)))
                                goalPt = mousePos
                        elif e.type == QUIT or (e.type == KEYUP
                                                and e.key == K_ESCAPE):
                            LOGGER.info("Leaving.")
                            return
            # convert mouse pos to Node
            if startPt is not None and self.startPt is None:
                self.startPt = Node(startPt)
            if goalPt is not None and self.goalPt is None:
                self.goalPt = Node(goalPt)
            self.update_screen(update_all=True)
        self.planner.add_newnode(self.startPt)

        ##################################################
        # calculate information regarding shortest path
        self.c_min = self.dist(self.startPt.pos, self.goalPt.pos)
        self.x_center = (self.startPt.pos[0] + self.goalPt.pos[0]) / 2, (
            self.startPt.pos[1] + self.goalPt.pos[1]) / 2
        dy = self.goalPt.pos[1] - self.startPt.pos[1]
        dx = self.goalPt.pos[0] - self.startPt.pos[0]
        self.angle = math.atan2(-dy, dx)

        self.planner.init(
            env=self,
            XDIM=self.XDIM,
            YDIM=self.YDIM,
            startPt=self.startPt,
            goalPt=self.goalPt,
            **kwargs)

    ############################################################

    def pygame_init(self, enable_pygame=True):
        self.args.enable_pygame = enable_pygame
        if not self.args.enable_pygame:
            return
        pygame.init()
        self.fpsClock = pygame.time.Clock()
        # self.fpsClock.tick(10)
        self.fpsClock.tick(10000)
        pygame.display.set_caption('RRTstar')
        # screen.fill(white)
        ################################################################################
        # text
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Arial',
                                          int(self.XDIM * 0.04 * self.args.scaling))
        ################################################################################
        # main window
        self.window = pygame.display.set_mode([
            int(self.XDIM * self.args.scaling),
            int((self.YDIM + self.extra) * self.args.scaling)
        ])
        ################################################################################
        # background aka the room
        self.background = pygame.Surface([self.XDIM, (self.YDIM + self.extra)])
        self.background.blit(self.img, (0, 0))
        # resize background to match windows
        self.background = pygame.transform.scale(self.background, [
            int(self.XDIM * self.args.scaling),
            int((self.YDIM + self.extra) * self.args.scaling)
        ])
        ################################################################################
        # path of RRT*
        self.path_layers = pygame.Surface([
            self.XDIM * self.args.scaling, (self.YDIM + self.extra) * self.args.scaling
        ])
        self.path_layers.fill(Colour.ALPHA_CK)
        self.path_layers.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the solution path
        self.solution_path_screen = pygame.Surface([
            self.XDIM * self.args.scaling, (self.YDIM + self.extra) * self.args.scaling
        ])
        self.solution_path_screen.fill(Colour.ALPHA_CK)
        self.solution_path_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the sampled points
        self.sampledPoint_screen = pygame.Surface([
            self.XDIM * self.args.scaling, (self.YDIM + self.extra) * self.args.scaling
        ])
        self.sampledPoint_screen.fill(Colour.ALPHA_CK)
        self.sampledPoint_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################

    @staticmethod
    def dist(p1, p2):
        # THIS IS MUCH SLOWER for small array
        # return np.linalg.norm(p1 - p2)
        p = p1 - p2;
        return math.sqrt(p[0] ** 2 + p[1] ** 2)

    def pygame_show(self):
        self.args.enable_pygame = True

    def pygame_hide(self):
        self.args.enable_pygame = False
        pygame.display.iconify()
        # pygame.quit()

    def step_from_to(self, p1, p2):
        """Get a new point from p1 to p2, according to step size."""
        if self.args.ignore_step_size:
            return p2
        if self.dist(p1, p2) < self.args.epsilon:
            return p2
        else:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            pos = p1[0] + self.args.epsilon * cos(
                theta), p1[1] + self.args.epsilon * sin(theta)
            return pos

    def run(self):
        """Run until we reached the specified max nodes"""
        while self.stats.valid_sample < self.args.max_number_nodes:
            self.process_pygame_event()
            self.update_screen()
            self.planner.run_once()
            # import time
            # time.sleep(.1)
        self.planner.terminates_hook()

    @check_pygame_enabled
    def draw_path(self,
                  node1,
                  node2,
                  colour=Colour.path_blue,
                  line_modifier=1,
                  layer=None):
        if layer is None:
            layer = self.path_layers
        pygame.draw.line(layer, colour, node1.pos * self.args.scaling,
                         node2.pos * self.args.scaling,
                         int(line_modifier * self.args.scaling))

    @check_pygame_enabled
    def draw_circle(self, pos, colour, radius, layer):
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    @check_pygame_enabled
    def process_pygame_event(self):
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                LOGGER.info("Leaving.")
                sys.exit(0)

    @check_pygame_enabled
    def wait_for_exit(self):
        while True:
            self.process_pygame_event()

############################################################
##                    DRAWING RELATED                     ##
############################################################

    @check_pygame_enabled
    def update_screen(self, update_all=False):
        if 'refresh_cnt' not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  #FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

###################################################################################
        def draw_start_goal_pt():
            if self.startPt is not None:
                self.draw_circle(
                    pos=self.startPt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)
            if self.goalPt is not None:
                self.draw_circle(
                    pos=self.goalPt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)

        # limites the screen update
        if count % 20 == 0:
            self.window.blit(self.background, (0, 0))

        if count % 60 == 0:
            try:
                self.planner.paint(window=self.window)
            except AttributeError as e:
                # print(e)
                pass
            draw_start_goal_pt()

        ##### Tree paths
        if count % 20 == 0:
            self.window.blit(self.path_layers, (0, 0))
            self.window.blit(self.solution_path_screen, (0, 0))
            draw_start_goal_pt()

        ##### Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.paint(window=self.window)
            except AttributeError as e:
                # print(e)
                pass

        ##### Sampled points
        if count % 4 == 0:
            self.sampledPoint_screen.fill(Colour.ALPHA_CK)
            # Draw sampled nodes
            for sampledPos in self.stats.sampledNodes:
                self.draw_circle(
                    pos=sampledPos,
                    colour=Colour.red,
                    radius=2,
                    layer=self.sampledPoint_screen)
            self.window.blit(self.sampledPoint_screen, (0, 0))
            # remove them from list
            del self.stats.sampledNodes[:]

        ##### Texts
        if count % 10 == 0:
            _cost = 'INF' if self.planner.c_max == float('inf') else round(
                self.planner.c_max, 2)
            if 'RRdTSampler' in self.args.sampler.__str__() and count > 0:
                num_nodes = sum(
                    len(tree.nodes) for tree in (
                        *self.args.sampler.tree_manager.disjointedTrees,
                        self.args.sampler.tree_manager.root))
            else:
                num_nodes = len(self.planner.nodes)
            # text = 'Cost_min: {}  | Nodes: {}'.format(_cost, num_nodes)
            # self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.YDIM * self.args.scaling * 0.88))
            text = 'Cost: {} | Inv.Samples: {}(con) {}(obs)'.format(
                _cost, self.stats.invalid_samples_connections,
                self.stats.invalid_samples_obstacles)
            self.window.blit(
                self.myfont.render(text, False, Colour.white, Colour.black),
                (10, (self.YDIM + self.extra) * self.args.scaling * 0.95))

        pygame.display.update()
