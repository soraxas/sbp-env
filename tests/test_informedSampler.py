from copy import deepcopy
from unittest import TestCase

import numpy as np

from env import Env
from samplers.informedSampler import InformedSampler
from tests.common_vars import template_args
from utils import planner_registry


class TestInformedSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = InformedSampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrt"]

        self.env = Env(**args)
        self.sampler = self.env.args.sampler

    def test_get_next_pos(self):
        # assert that the sampling points returned after getting an initial solution
        # has a smaller range

        np.random.seed(0)

        pts_before_sol = []
        for i in range(100):
            pts_before_sol.append(self.sampler.get_next_pos()[0])
        pts_before_sol = np.array(pts_before_sol)

        self.sampler.args.planner.c_max = 80

        pts_after_sol = []
        for i in range(100):
            pts_after_sol.append(self.sampler.get_next_pos()[0])
        pts_after_sol = np.array(pts_after_sol)

        self.assertTrue((pts_before_sol.max(axis=0) > pts_after_sol.max(axis=0)).all())
