"""Represent a planner."""
from planners import rrtPlanner
from samplers import informedrrtSampler
from utils import planner_registry

# start register
planner_registry.register_planner(
    "informedrrt",
    planner_class=rrtPlanner.RRTPlanner,
    visualise_pygame_paint=rrtPlanner.pygame_rrt_paint,
    sampler_id=informedrrtSampler.sampler_id,
)
# finish register
