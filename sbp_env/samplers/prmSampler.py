from overrides import overrides

from ..samplers.randomPolicySampler import RandomPolicySampler
from ..utils import planner_registry


class PRMSampler(RandomPolicySampler):
    r"""This sampler is basically the same as
    :class:`samplers.randomPolicySampler.RandomPolicySampler`, excepts we
    always overrides the goal bias epsilon :math:`\epsilon=0` as PRM could not
    utilise the benefit of goal bias.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method"""
        kwargs["goalBias"] = 0
        super().init(**kwargs)


# start register
sampler_id = "prm_sampler"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=PRMSampler,
)
# finish register
