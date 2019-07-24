"""Represent a planner."""
import random
import math
import numpy as np
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
        self.cMin = self.args.env.dist(self.start_pos, self.goal_pos) - self.args.goal_radius
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
