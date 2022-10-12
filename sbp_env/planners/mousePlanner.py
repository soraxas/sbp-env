from ..planners import rrtPlanner
from ..samplers import mouseSampler
from ..utils import planner_registry

# start register
planner_registry.register_planner(
    "mouse",
    planner_class=rrtPlanner.RRTPlanner,
    visualise_pygame_paint=rrtPlanner.pygame_rrt_paint,
    sampler_id=mouseSampler.sampler_id,
)
# finish register
