import random

from overrides import overrides

from samplers.baseSampler import Sampler
from samplers.randomPolicySampler import RandomPolicySampler
from utils import planner_registry


# noinspection PyAttributeOutsideInit
class BiRRTSampler(Sampler):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(use_radian=self.use_radian, **kwargs)

    def set_use_radian(self, value=True):
        self.use_radian = value
        self.randomSampler.use_radian = value

    @overrides
    def get_next_pos(self):
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
