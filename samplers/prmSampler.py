from overrides import overrides

from samplers.randomPolicySampler import RandomPolicySampler
from utils import planner_registry


class PRMSampler(RandomPolicySampler):
    """ """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        """

        :param **kwargs: 

        """
        kwargs["goalBias"] = 0
        super().init(**kwargs)


sampler_id = "prm_sampler"

planner_registry.register_sampler(
    sampler_id, sampler_class=PRMSampler,
)
