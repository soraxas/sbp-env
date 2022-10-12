from io import BytesIO

import numpy as np
from PIL import Image

from sbp_env import engine
from sbp_env.planners.basePlanner import Planner
from sbp_env.samplers.baseSampler import Sampler
from sbp_env.utils.common import MagicDict, Stats
from sbp_env.utils.planner_registry import PlannerDataPack, SamplerDataPack
from sbp_env.visualiser import VisualiserSwitcher

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
    epsilon=10.0,
    goalBias=0.02,
    start_pt="100,100",
    goal_pt="350,350",
    goal_radius=8.0,
    ignore_step_size=False,
    max_number_nodes=10000,
    no_display=True,
    planner_data_pack=PlannerDataPack(
        name="my_planner_id",
        planner_class=DummyPlannerClass,
        sampler_id="my_sampler_id",
        visualise_pygame_paint_init=None,
        visualise_pygame_paint=None,
        visualise_pygame_paint_terminate=None,
        visualise_klampt_paint_init=None,
        visualise_klampt_paint=None,
        visualise_klampt_paint_terminate=None,
    ),
    sampler_data_pack=SamplerDataPack(
        name="my_sampler_id",
        sampler_class=DummySamplerClass,
        visualise_pygame_paint_init=None,
        visualise_pygame_paint=None,
        visualise_pygame_paint_terminate=None,
        visualise_klampt_paint_init=None,
        visualise_klampt_paint=None,
        visualise_klampt_paint_terminate=None,
    ),
    planner_type=DummyPlannerClass,
    radius=12.0,
    rrdt_proposal_distribution="dynamic-vonmises",
    scaling=1.5,
    showSampledPoint=False,
    skip_optimality=False,
    stats=Stats(),
)
template_args["engine"] = engine.ImageEngine(template_args, "maps/room1.png")


# helper class to test equality for numpy array
class MockNumpyEquality:
    def __init__(self, target, almost_equal=False):
        self.target = target
        self.check_almost_equal = almost_equal

    def __eq__(self, other):
        if not isinstance(other, np.ndarray):
            return False
        if self.check_almost_equal:
            np.testing.assert_allclose(self.target, other)
        else:
            np.testing.assert_array_equal(self.target, other)
        return True
