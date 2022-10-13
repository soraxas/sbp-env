from copy import deepcopy
from unittest import TestCase

import numpy as np

from sbp_env import visualiser
from sbp_env.env import Env
from sbp_env.samplers.informedSampler import InformedSampler
from sbp_env.utils import planner_registry
from tests.common_vars import template_args


class TestInformedSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args.sampler_data_pack.sampler_class = InformedSampler

        # use some suitable planner
        args.planner_data_pack = planner_registry.PLANNERS["rrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler

    def test_get_next_pos(self):
        # assert that the sampling points returned after getting an initial solution
        # has a smaller range

        np.random.seed(0)

        pts_before_sol = []
        for i in range(100):
            pts_before_sol.append(self.sampler.get_next_pos()[0])
        pts_before_sol = np.array(pts_before_sol)

        self.sampler.args.planner.c_max = (
            np.linalg.norm(self.env.start_pt.pos - self.env.goal_pt.pos) + 1
        )

        pts_after_sol = []
        for i in range(100):
            pts_after_sol.append(self.sampler.get_next_pos()[0])
        pts_after_sol = np.array(pts_after_sol)

        self.assertTrue((pts_before_sol.max(axis=0) > pts_after_sol.max(axis=0)).all())
