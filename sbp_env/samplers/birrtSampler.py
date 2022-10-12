import random

from overrides import overrides

from ..samplers.baseSampler import Sampler
from ..samplers.randomPolicySampler import RandomPolicySampler
from ..utils import planner_registry


# noinspection PyAttributeOutsideInit
class BiRRTSampler(Sampler):
    r"""
    The sampler that is used internally by :class:`~planners.birrtPlanner.BiRRTPlanner`.
    Internally, :class:`~samplers.birrtSampler.BiRRTSampler`
    uses :class:`~samplers.randomPolicySampler.RandomPolicySampler` to draw from its
    supported random methods.
    The main differences lies in the epsilon biasing when

    .. math::

        p \sim \mathcal{U}(0,1) < \epsilon,

    where the sampler will bias towards the correct **start** or **goal** tree
    depending on the current tree :math:`\mathcal{T}_\text{current}` that
    :class:`~samplers.birrtSampler.BiRRTSampler`
    is currently planning for (in contrast to only always biasing towards the goal tree).

    That is, :math:`p \sim \mathcal{U}(0,1)` is first drawn, then :math:`q_\text{new}`
    is given by

    .. math::
        q_\text{new} =
        \begin{cases}
            q \sim \mathcal{U}(0,1)^d & \text{if } p < \epsilon\\
            q_\text{target}  & \text{if } \mathcal{T}_\text{current} \equiv \mathcal{
            T}_{start}\\
            q_\text{start}  &  \text{if } \mathcal{T}_\text{current} \equiv \mathcal{
            T}_{target}\text{.}
        \end{cases}

    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.random_method = (
            kwargs["random_method"] if "random_method" in kwargs else "pseudo_random"
        )

    @overrides
    def init(self, **kwargs):
        """The delayed **initialisation** method"""
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler(random_method=self.random_method)
        self.randomSampler.init(use_radian=self.use_radian, **kwargs)

    def set_use_radian(self, value=True):
        """Overrides the super class method such that the value will be passed to the
        internal :class:`samplers.randomPolicySampler.RandomPolicySampler`

        :param value: whether to use radian

        """
        self.use_radian = value
        self.randomSampler.use_radian = value

    @overrides
    def get_next_pos(self) -> Sampler.GetNextPosReturnType:
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


# start register
planner_registry.register_sampler(
    "birrt_sampler",
    sampler_class=BiRRTSampler,
)
# finish register
