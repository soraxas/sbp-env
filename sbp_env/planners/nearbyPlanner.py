from ..planners import rrtPlanner
from ..samplers import nearbyPolicySampler
from ..utils import planner_registry

# start register
planner_registry.register_planner(
    "nearby",
    planner_class=rrtPlanner.RRTPlanner,
    visualise_pygame_paint=rrtPlanner.pygame_rrt_paint,
    sampler_id=nearbyPolicySampler.sampler_id,
)
# finish register
