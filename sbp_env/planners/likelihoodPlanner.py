from ..planners import rrtPlanner
from ..samplers import likelihoodPolicySampler
from ..utils import planner_registry

# start register
planner_registry.register_planner(
    "likelihood",
    planner_class=rrtPlanner.RRTPlanner,
    visualise_pygame_paint=rrtPlanner.pygame_rrt_paint,
    sampler_id=likelihoodPolicySampler.sampler_id,
)
# finish register
