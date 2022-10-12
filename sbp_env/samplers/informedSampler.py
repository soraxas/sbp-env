"""Represent a planner."""
import math
import random

import numpy as np
from overrides import overrides

from ..samplers.baseSampler import Sampler
from ..samplers.randomPolicySampler import RandomPolicySampler
from ..utils import planner_registry
from ..utils.common import Colour


# noinspection PyAttributeOutsideInit
class InformedSampler(Sampler):
    r"""The informed sampler is largely similar to
    :class:`~samplers.randomPolicySampler.RandomPolicySampler`, excepts when an
    initial solution is found (i.e. when the current maximum cost :math:`\mathcal{
    L}_\text{max} < \infty`), the
    sampler will uses ellipsoidal heuristic to speed up convergence of solution cost.

    The sampled configuratioin :math:`q_\text{new}` is given by

    .. math::
        q_\text{new} =
        \begin{cases}
            \mathbf{C}\,\mathbf{L}\,q_\text{ball} + q_\text{center} & \text{if }
            \mathcal{L}_\text{
            max} <
            \infty\\
            q \sim \mathcal{U}(0,1)^d  & \text{otherwise,}
        \end{cases}

    where :math:`q_\text{ball} \sim \mathcal{U}(\mathcal{Q}_\text{ball})` is a
    uniform sample drawn from a unit :math:`n`-ball, :math:`q_\text{center} = \frac{
    q_\text{start} + q_\text{start}}{2}` is the center location in-between the start
    and goal configuration, :math:`\mathbf{C} \in SO(d)` is the rotational matrix to
    transforms from the hyperellipsoid frame to the world frame,

    .. math::
        \mathbf{L} = \operatorname{diag}\left\{
            \frac{\mathcal{L}_\text{max}}{2},
            \frac{\sqrt{\mathcal{L}^2_\text{max} -\mathcal{L}^2_\text{min}}}{2},
            \ldots,
            \frac{\sqrt{\mathcal{L}^2_\text{max} -\mathcal{L}^2_\text{min}}}{2}
        \right\}

    is the diagonal transformation matrix to maintain uniform distribution in the
    ellipse space, and the minimum cost is given by

    .. math::
        \mathcal{L}_\text{min} =
            \lVert q_\text{start} -q_\text{target}\rVert_2 .

    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method"""
        super().init(**kwargs)
        self.random_sampler = RandomPolicySampler()
        self.random_sampler.init(**kwargs)

        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        self.cBest = float("inf")

        # Computing the sampling space
        self.cMin = (
            self.args.engine.dist(self.start_pos, self.goal_pos) - self.args.goal_radius
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
        """Retrieve next sampled position"""
        if self.args.engine == "klampt":
            # not possible with radian space
            p = self.random_sampler.get_next_pos()[0]
            return p, self.report_success, self.report_fail

        self.cBest = self.args.planner.c_max
        if self.cBest < float("inf"):
            r = [
                self.cBest / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0,
            ]
            L = np.diag(r)
            xBall = self.sample_unit_ball()
            rnd = np.dot(np.dot(self.C, L), xBall) + self.xCenter
            p = [rnd[(0, 0)], rnd[(1, 0)]]
            if self.args.engine == "4d":
                p.extend(np.random.uniform([-np.pi, -np.pi], [np.pi, np.pi]))
        else:
            p = self.random_sampler.get_next_pos()[0]
        return p, self.report_success, self.report_fail

    @staticmethod
    def sample_unit_ball() -> np.ndarray:
        """Samples a unit :math:`n`-ball

        :return: a random samples from a unit :math:`n`-ball
        """
        a = random.random()
        b = random.random()
        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b), b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])


def pygame_informed_sampler_paint(sampler: Sampler) -> None:
    """Visualiser paint function for informed sampler

    :param sampler: the sampler to be visualised

    """
    import pygame

    # draw the ellipse
    if sampler.cBest < float("inf"):
        cBest = sampler.cBest * sampler.args.scaling
        cMin = sampler.cMin * sampler.args.scaling
        a = math.sqrt(cBest**2 - cMin**2)  # height
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
        ellipse_x = sampler.xCenter[0][0] * sampler.args.scaling - rcx
        ellipse_y = sampler.xCenter[1][0] * sampler.args.scaling - rcy
        sampler.args.env.window.blit(ellipse_surface, (ellipse_x, ellipse_y))


# start register
sampler_id = "informed_sampler"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=InformedSampler,
    visualise_pygame_paint=pygame_informed_sampler_paint,
)
# finish register
