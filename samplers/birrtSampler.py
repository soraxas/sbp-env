import random
from typing import Tuple, Callable

import numpy as np
from overrides import overrides

from samplers.baseSampler import Sampler
from samplers.randomPolicySampler import RandomPolicySampler
from utils import planner_registry


# noinspection PyAttributeOutsideInit
class BiRRTSampler(Sampler):
    """The sampler that is used internally in
    :class:`planners.birrtPlanner.BiRRTPlanner`"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method

        :param **kwargs:

        """
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(use_radian=self.use_radian, **kwargs)

    def set_use_radian(self, value=True):
        """Overrides the super class method such that the value will be passed to the
        internal :class:`samplers.randomPolicySampler.RandomPolicySampler`

        :param value: whether to use radian

        """
        self.use_radian = value
        self.randomSampler.use_radian = value

    @overrides
    def get_next_pos(self) -> Tuple[np.ndarray, Callable, Callable]:
        """Get next sampled position"""
        # Random path
        while True:
            if random.random() < self.args.goalBias:
                # init/goal bias
                if self.args.planner.goal_tree_turn:
                    p = self.start_pos
                else:
                    p = self.goal_pos
            else:
                p = self.randomSampler.get_next_pos()[0]
            return p, self.report_success, self.report_fail


planner_registry.register_sampler(
    "birrt_sampler", sampler_class=BiRRTSampler,
)
