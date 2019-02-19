"""Represent a planner."""
import random

import numpy as np

import pygame
from checkCollision import *
from helpers import *
from overrides import overrides
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler
from planners.rrtPlanner import RRTPlanner


class InformedRRTSampler(Sampler):
    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)

        # max length we expect to find in our 'informed' sample space, starts as infinite
        self.cBest = float('inf')
        pathLen = float('inf')
        solutionSet = set()
        path = None

        # Computing the sampling space
        self.cMin = math.sqrt(
            pow(self.start_pos[0] - self.goal_pos[0], 2) +
            pow(self.start_pos[1] - self.goal_pos[1], 2))
        self.xCenter = np.array(
            [[(self.start_pos[0] + self.goal_pos[0]) / 2.0],
             [(self.start_pos[1] + self.goal_pos[1]) / 2.0], [0]])
        a1 = np.array([[(self.goal_pos[0] - self.start_pos[0]) / self.cMin],
                       [(self.goal_pos[1] - self.start_pos[1]) / self.cMin],
                       [0]])

        self.etheta = math.atan2(a1[1], a1[0])
        # first column of idenity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, 1, 1)
        self.C = np.dot(
            np.dot(U, np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))
                ])), Vh)

    @overrides
    def get_next_pos(self):
        # Random path
        self.cBest = self.args.planner.c_max
        # print('max: {}'.format(self.cBest))
        # print('min: {}'.format(self.cMin))
        # self.cBest = self.c_max
        if self.cBest < float('inf'):
            r = [
                self.cBest / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0
            ]
            L = np.diag(r)
            xBall = self.sampleUnitBall()
            rnd = np.dot(np.dot(self.C, L), xBall) + self.xCenter
            p = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            p = self.randomSampler.get_next_pos()[0]
        return p, self.report_success, self.report_fail

    @staticmethod
    def sampleUnitBall():
        a = random.random()
        b = random.random()
        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])


class InformedRRTPlanner(RRTPlanner):
    """This planner is largely a RRT planner, though with extra features."""

    @overrides
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


    @overrides
    def paint(self):
        super().paint()
        if self.args.sampler.cBest == float('inf'):
            return
        # paint the ellipse
        cBest = self.args.sampler.cBest * self.args.scaling
        cMin = self.args.sampler.cMin * self.args.scaling
        a = math.sqrt(cBest**2 - cMin**2) # height
        b = cBest # width

        # rectangle that represent the ellipse
        r = pygame.Rect(0, 0, b, a)
        angle = self.args.sampler.etheta
        # rotate via surface
        ellipse_surface = pygame.Surface((b, a), pygame.SRCALPHA, 32).convert_alpha()
        pygame.draw.ellipse(ellipse_surface, Colour.black, r, 3)
        # rotate
        ellipse_surface = pygame.transform.rotate(ellipse_surface, -angle * 180 / math.pi)

        # r.centerx = self.args.sampler.xCenter[0] * self.args.scaling
        # r.centery = self.args.sampler.xCenter[1] * self.args.scaling


        # we need to offset the blitz based on the surface ceenter
        rcx, rcy = ellipse_surface.get_rect().center
        ellipse_x = (self.args.sampler.xCenter[0] * self.args.scaling - rcx)
        ellipse_y = (self.args.sampler.xCenter[1] * self.args.scaling - rcy)

        self.args.env.solution_path_screen.blit(ellipse_surface, (ellipse_x, ellipse_y))






        # self.args.env.window.blit(r, (0, 0))
