"""Represent a planner."""
import math
import random

import numpy as np
from overrides import overrides

from planners import rrtPlanner
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler
from utils import planner_registry


# noinspection PyAttributeOutsideInit
class InformedRRTSampler(Sampler):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.random_sampler = RandomPolicySampler()
        self.random_sampler.init(**kwargs)

        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        self.cBest = float("inf")

        # Computing the sampling space
        self.cMin = (
            self.args.env.dist(self.start_pos, self.goal_pos) - self.args.goal_radius
        )
        self.xCenter = np.array(
            [
                [(self.start_pos[0] + self.goal_pos[0]) / 2.0],
                [(self.start_pos[1] + self.goal_pos[1]) / 2.0],
                [0],
            ]
        )
        a1 = np.array(
            [
                [(self.goal_pos[0] - self.start_pos[0]) / self.cMin],
                [(self.goal_pos[1] - self.start_pos[1]) / self.cMin],
                [0],
            ]
        )

        self.etheta = math.atan2(a1[1], a1[0])
        # first column of identity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, True, True)
        self.C = np.dot(
            np.dot(
                U,
                np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))]),
            ),
            Vh,
        )

    @overrides
    def get_next_pos(self):
        if self.args.engine == "klampt":
            # not possible with radian space
            p = self.random_sampler.get_next_pos()[0]
            return p, self.report_success, self.report_fail

        self.cBest = self.args.planner.c_max
        if self.cBest < float("inf"):
            r = [
                self.cBest / 2.0,
                math.sqrt(self.cBest ** 2 - self.cMin ** 2) / 2.0,
                math.sqrt(self.cBest ** 2 - self.cMin ** 2) / 2.0,
            ]
            L = np.diag(r)
            xBall = self.sampleUnitBall()
            rnd = np.dot(np.dot(self.C, L), xBall) + self.xCenter
            p = [rnd[(0, 0)], rnd[(1, 0)]]
            if self.args.image == "maps/4d.png":
                p.extend(np.random.uniform([-np.pi, -np.pi], [np.pi, np.pi]))
        else:
            p = self.random_sampler.get_next_pos()[0]
        return p, self.report_success, self.report_fail

    @staticmethod
    def sampleUnitBall():
        a = random.random()
        b = random.random()
        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b), b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])


def pygame_informed_sampler_paint(sampler):
    import pygame
    from utils.helpers import Colour

    # draw the ellipse
    if sampler.cBest < float("inf"):
        cBest = sampler.cBest * sampler.args.scaling
        cMin = sampler.cMin * sampler.args.scaling
        a = math.sqrt(cBest ** 2 - cMin ** 2)  # height
        b = cBest  # width
        # rectangle that represent the ellipse
        r = pygame.Rect(0, 0, b, a)
        angle = sampler.etheta
        # rotate via surface
        ellipse_surface = pygame.Surface((b, a), pygame.SRCALPHA, 32).convert_alpha()
        try:
            pygame.draw.ellipse(ellipse_surface, (255, 0, 0, 80), r)
            pygame.draw.ellipse(
                ellipse_surface, Colour.black, r, int(2 * sampler.args.scaling)
            )
        except ValueError:
            # sometime it will fail to draw due to ellipse being too narrow
            pass
        # rotate
        ellipse_surface = pygame.transform.rotate(
            ellipse_surface, -angle * 180 / math.pi
        )
        # we need to offset the blitz based on the surface ceenter
        rcx, rcy = ellipse_surface.get_rect().center
        ellipse_x = sampler.xCenter[0] * sampler.args.scaling - rcx
        ellipse_y = sampler.xCenter[1] * sampler.args.scaling - rcy
        sampler.args.env.window.blit(ellipse_surface, (ellipse_x, ellipse_y))


planner_registry.register_sampler(
    "informed_sampler",
    sampler_class=InformedRRTSampler,
    visualise_pygame_paint=pygame_informed_sampler_paint,
)

planner_registry.register_planner(
    "informedrrt",
    planner_class=rrtPlanner.RRTPlanner,
    visualise_pygame_paint=rrtPlanner.pygame_rrt_paint,
    sampler_id="informed_sampler",
)
