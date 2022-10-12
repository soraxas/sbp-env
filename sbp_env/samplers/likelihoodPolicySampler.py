import logging
import random

import numpy as np
import scipy as sp
import scipy.ndimage
from overrides import overrides

from ..collisionChecker import ImgCollisionChecker
from ..samplers.baseSampler import Sampler
from ..samplers.randomPolicySampler import RandomPolicySampler
from ..utils import planner_registry

LOGGER = logging.getLogger(__name__)

experimental_sampler_note = {
    "only_work_with_2d_image": r"""This sampler currently only works with **2D Image 
        Space** as it's hard-coded to sub-divide the space into grid cells. However, 
        the idea is generic and should be applicable for :math:`d > 2`.""",
    "currently_expr_for_research": r"""This sampler is experimental for research 
        purpose, and is currently incomplete.""",
}


# noinspection PyAttributeOutsideInit
class LikelihoodPolicySampler(Sampler):
    __doc__ = r"""This sampler discretise *C-Space* into equal cells, then each cells is
    consists of a probability value :math:`p` that stores a likelihood value.

    The probability :math:`p` of a cell will be increased if
    :math:`q \in C_\text{free}` is reported but the tree extension is failed due to visibility test
    (intermediate obstacles). The probability :math:`p` will be dropped depending on
    its proximity to an existing tree node.
    """ + r"""
    .. note::
        {only_work_with_2d_image}

    .. note::
        {currently_expr_for_research}

    """.format(
        **experimental_sampler_note
    )

    @overrides
    def __init__(
        self, prob_block_size: int, suppress_visited_area: bool = True, **kwargs
    ):
        """
        :param prob_block_size: the unit size for one probability block
        :param suppress_visited_area: reduce the probability :math:`p` if it is close
            to an existing tree node
        """
        super().__init__(**kwargs)
        self.PROB_BLOCK_SIZE = int(prob_block_size)
        self.suppress_visited_area = suppress_visited_area

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method"""
        super().init(**kwargs)
        self.random_sampler = RandomPolicySampler()
        self.random_sampler.init(**kwargs)

        self.shape = (
            int(self.args.engine.upper[0] / self.PROB_BLOCK_SIZE) + 1,
            int(self.args.engine.upper[1] / self.PROB_BLOCK_SIZE) + 1,
        )
        self.prob_vector = np.ones(self.shape)
        self.prob_vector *= 1  # IMPORTANT because we are using log2
        self.obst_vector = np.ones(self.shape)
        # self.prob_vector *= 20
        self.prob_vector_normalized = None
        self.tree_vector = np.ones(self.shape)

        self.sampleCount = 0

    @overrides
    def get_next_pos(self) -> Sampler.GetNextPosReturnType:
        if self.prob_vector_normalized is None or random.random() < 0.05:
            p = self.random_sampler.get_next_pos()[0]
        else:
            choice = np.random.choice(
                range(self.prob_vector_normalized.size),
                p=self.prob_vector_normalized.ravel(),
            )
            y = choice % self.prob_vector_normalized.shape[1]
            x = int(choice / self.prob_vector_normalized.shape[1])
            p = (
                (x + random.random()) * self.PROB_BLOCK_SIZE,
                (y + random.random()) * self.PROB_BLOCK_SIZE,
            )
        return p, self.report_success, self.report_fail

    @overrides
    def add_tree_node(self, pos: np.ndarray, **kwargs):
        """Report that a new tree node has been created

        :param pos: the configuration of the new tree node

        """
        x, y = pos
        x = int(x / self.PROB_BLOCK_SIZE)
        y = int(y / self.PROB_BLOCK_SIZE)
        self.tree_vector[x][y] += 1

    @overrides
    def add_sample_line(self, x1: int, y1: int, x2: int, y2: int):
        """Add a 2D sample line to denotes from a visibility line that are all in free
        space

        :param x1: :math:`x` of :math:`q_1`
        :param y1: :math:`y` of :math:`q_1`
        :param x2: :math:`x` of :math:`q_2`
        :param y2: :math:`y` of :math:`q_2`

        """
        x1 = int(x1 / self.PROB_BLOCK_SIZE)
        y1 = int(y1 / self.PROB_BLOCK_SIZE)
        x2 = int(x2 / self.PROB_BLOCK_SIZE)
        y2 = int(y2 / self.PROB_BLOCK_SIZE)
        points = ImgCollisionChecker.get_line((x1, y1), (x2, y2))
        for p in points:
            self.report_fail(pos=p, free=True, alreadyDividedByProbBlockSize=True)

    @overrides
    def report_success(self, **kwargs):
        """Report a successful sample

        :param pos: the position of a newly created node
        :param nn: the nearest existing node
        :param rand_pos: the position of a successful random sample

        :type pos: numpy.ndarray
        :type nn: Node
        :type rand_pos: numpy.ndarray

        """
        x, y = kwargs["pos"]
        # add all in between point of nearest node of the random pt as valid
        x1, y1 = self.args.engine.cc.get_coor_before_collision(
            kwargs["nn"].pos, kwargs["rand_pos"]
        )
        self.add_sample_line(x, y, x1, y1)

    @overrides
    def report_fail(self, **kwargs):
        """Report a failed sample

        :param pos: the position of the random sample

        :type pos: numpy.ndarray

        """
        p = kwargs["pos"]
        if p is None:
            return
        try:
            p = p.pos
        except AttributeError as e:
            pass
        if "alreadyDividedByProbBlockSize" not in kwargs:
            x = int(p[0] / self.PROB_BLOCK_SIZE)
            y = int(p[1] / self.PROB_BLOCK_SIZE)
        else:
            x = p[0]
            y = p[1]
        if (
            x < 0
            or x >= self.prob_vector.shape[0]
            or y < 0
            or y >= self.prob_vector.shape[1]
        ):
            return

        # exit()
        # ^ setting the right coordinators ^
        ###############################
        return self._report_fail_impl(x, y, **kwargs)

    def _report_fail_impl(self, x: int, y: int, **kwargs):
        r"""The internal implantation of the :func:`report_fail`.

        :param x: :math:`x` of :math:`q`
        :param y: :math:`y` of :math:`q`
        :param weight: the weight for this sample
        :param free: whether it failed due to :math:`q \in C_\text{free}` or
            :math:`q \in C_\text{obs}`

        :type weight: float, optional
        :type free: bool

        """
        if "obstacle" in kwargs:
            self.obst_vector[x][y] += 2
        elif not kwargs["free"]:
            self.obst_vector[x][y] += 1
            # self.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
            # if self.prob_vector[x][y] < 5:
            # self.prob_vector[x][y] = 5
        elif kwargs["free"]:
            if "weight" in kwargs:
                self.prob_vector[x][y] += kwargs["weight"]
            else:
                self.prob_vector[x][y] += 1
                # self.prob_vector[x][y] = 10
            self.obst_vector[x][y] = 1

            #########################################################

            sigma_y = 2.0
            sigma_x = 2.0
            sigma = [sigma_y, sigma_x]
            if self.sampleCount % 10 == 0:
                pass
                self.prob_vector_normalized = np.copy(self.prob_vector)
                # if self.prob_vector[x][y] < 5:
                # self.prob_vector[copy(self.prob_vector)
                tree_vector_normalized = np.copy(self.tree_vector**1.1)
                tree_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    tree_vector_normalized, (1.0, 1.0), mode="reflect"
                )
                # self.prob_vector_normalized = tree_vector_normalized
                self.prob_vector_normalized *= 1 / self.obst_vector * 3
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    self.prob_vector_normalized, sigma, mode="reflect"
                )
                self.prob_vector_normalized *= 1 / tree_vector_normalized * 1.5
                # self.prob_vector_normalized *= (1/self.tree_vector * 1.5)
                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            self.sampleCount += 1


def pygame_likelihood_sampler_paint_init(sampler):
    """The paint int function for visualisation

    :param sampler: the sampler to be visualised

    """
    import pygame

    # probability layer
    sampler.prob_layer = pygame.Surface(
        (
            sampler.PROB_BLOCK_SIZE * sampler.args.scaling,
            sampler.PROB_BLOCK_SIZE * sampler.args.scaling,
        ),
        pygame.SRCALPHA,
    )


def pygame_likelihood_sampler_paint(sampler):
    """The paint function for visualisation

    :param sampler: the sampler to be visualised

    """

    def get_vector_alpha_parameters(vector):
        """

        :param vector:

        """
        _max_prob = vector.max()
        _min_prob = vector.min()
        _denominator = _max_prob - _min_prob
        if _denominator == 0:
            _denominator = 1  # prevent division by zero
        return _max_prob, _min_prob, _denominator

    if sampler.prob_vector_normalized is not None:
        for i in range(sampler.prob_vector_normalized.shape[0]):
            for j in range(sampler.prob_vector_normalized.shape[1]):
                max_prob, min_prob, denominator = get_vector_alpha_parameters(
                    sampler.prob_vector_normalized
                )
                alpha = 240 * (
                    1 - (sampler.prob_vector_normalized[i][j] - min_prob) / denominator
                )
                sampler.prob_layer.fill((255, 128, 255, alpha))
                # print(sampler.prob_vector_normalized[i][j])
                sampler.args.env.window.blit(
                    sampler.prob_layer,
                    (
                        i * sampler.PROB_BLOCK_SIZE * sampler.args.scaling,
                        j * sampler.PROB_BLOCK_SIZE * sampler.args.scaling,
                    ),
                )


# start register
sampler_id = "likelihood_sampler"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=LikelihoodPolicySampler,
    visualise_pygame_paint=pygame_likelihood_sampler_paint,
    visualise_pygame_paint_init=pygame_likelihood_sampler_paint_init,
)
# finish register
