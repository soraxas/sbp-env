from io import BytesIO

import numpy as np
from PIL import Image

from planners.basePlanner import Planner
from planners.rrtPlanner import RRTPlanner
from samplers.baseSampler import Sampler
from utils.common import MagicDict
from utils.planner_registry import PlannerDataPack
from visualiser import VisualiserSwitcher

mock_image_as_np = np.array(
    [
        [255, 255, 0, 0, 100, 200],
        [255, 255, 0, 0, 100, 200],
        [255, 255, 0, 0, 100, 200],
        [255, 255, 0, 0, 100, 200],
    ]
)


def create_test_image():
    file = BytesIO()
    image = Image.fromarray(mock_image_as_np.astype(np.uint8))
    image.save(file, "png")
    file.name = "test.png"
    file.seek(0)
    return file


class DummyPlannerClass(Planner):
    pass


class DummySamplerClass(Sampler):
    pass


# disable visualisation
VisualiserSwitcher.choose_visualiser("base")

# common args
template_args = MagicDict(
    always_refresh=False,
    engine="image",
    epsilon=10.0,
    goalBias=0.02,
    start_pt="0,0",
    goal_pt="1,1",
    num_dim=2,
    goal_radius=8.0,
    ignore_step_size=False,
    image="maps/room1.png",
    max_number_nodes=10000,
    no_display=True,
    planner_data_pack=PlannerDataPack(
        name="my_planner_id",
        planner_class=DummyPlannerClass,
        sampler_id="my_sampler_id",
        visualise_pygame_paint_init=None,
        visualise_pygame_paint=None,
        visualise_pygame_paint_terminate=None,
    ),
    planner_type=DummyPlannerClass,
    radius=12.0,
    rrdt_proposal_distribution="dynamic-vonmises",
    sampler=DummySamplerClass(),
    scaling=1.5,
    showSampledPoint=False,
    skip_optimality=False,
)
