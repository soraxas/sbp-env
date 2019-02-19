"""Represent a planner."""
import random
import numpy as np
import pygame
from checkCollision import *
from helpers import *
from overrides import overrides
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler


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
        self.cMin = dist(self.start_pos, self.goal_pos) - self.args.goal_radius
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
        self.cBest = self.args.planner.c_max
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

    def paint(self, window):
        # draw the ellipse
        if self.args.sampler.cBest < float('inf'):
            cBest = self.cBest * self.args.scaling
            cMin = self.cMin * self.args.scaling
            a = math.sqrt(cBest**2 - cMin**2) # height
            b = cBest # width

            # rectangle that represent the ellipse
            r = pygame.Rect(0, 0, b, a)
            angle = self.etheta
            # rotate via surface
            ellipse_surface = pygame.Surface((b, a), pygame.SRCALPHA, 32).convert_alpha()
            pygame.draw.ellipse(ellipse_surface, Colour.black, r, int(2 * self.args.scaling))
            # rotate
            ellipse_surface = pygame.transform.rotate(ellipse_surface, -angle * 180 / math.pi)

            # we need to offset the blitz based on the surface ceenter
            rcx, rcy = ellipse_surface.get_rect().center
            ellipse_x = (self.xCenter[0] * self.args.scaling - rcx)
            ellipse_y = (self.xCenter[1] * self.args.scaling - rcy)

            window.blit(ellipse_surface, (ellipse_x, ellipse_y))
            # self.args.env.window.blit(ellipse_surface, (ellipse_x, ellipse_y))
