import random

import numpy as np
from overrides import overrides

from ..randomness import SUPPORTED_RANDOM_METHODS, RandomnessManager
from ..samplers.baseSampler import Sampler
from ..utils import planner_registry


class RandomPolicySampler(Sampler):
    r"""Uniformly and randomly samples configurations across :math:`d` where
    :math:`d` is
    the dimensionality of the *C-Space*.
    :class:`~samplers.randomPolicySampler.RandomPolicySampler` samples configuration
    :math:`q \in \mathbb{R}^d` across each dimension uniformly with an
    :math:`0 \le \epsilon < 1` bias towds the goal configuration.

    A random number :math:`p \sim \mathcal{U}(0,1)` is first drawn, then the
    configuration :math:`q_\text{new}` that this function returns is given by

    .. math::
        q_\text{new} =
        \begin{cases}
            q \sim \mathcal{U}(0,1)^d & \text{if } p < \epsilon\\
            q_\text{target}  & \text{otherwise.}
        \end{cases}

    :py:const:`CONSTANT`
    """

    @overrides
    def __init__(self, random_method: str = "pseudo_random", **kwargs):
        """
        :param random_method: the kind of random method to use. Must be a choice from
            :data:`randomness.SUPPORTED_RANDOM_METHODS`.
        :param kwargs: pass through to super class
        """
        super().__init__(**kwargs)
        if random_method not in SUPPORTED_RANDOM_METHODS:
            raise ValueError(
                "Given random_method is not valid! Valid options includes:\n"
                "{}".format(
                    "\n".join((" - {}".format(m) for m in SUPPORTED_RANDOM_METHODS))
                )
            )

        self.random_method = random_method
        self.random = None

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method"""
        super().init(**kwargs)
        self.random = RandomnessManager(num_dim=self.args.engine.get_dimension())

        self.use_original_method = False

    @overrides
    def get_next_pos(self) -> Sampler.GetNextPosReturnType:
        # Random path
        if random.random() < self.args.goalBias:
            # goal bias
            p = self.goal_pos
        else:
            p = self.random.get_random(self.random_method)
            p = self.args.engine.transform(p)

        return p, self.report_success, self.report_fail


# start register
sampler_id = "random"

planner_registry.register_sampler(sampler_id, sampler_class=RandomPolicySampler)
# finish register
