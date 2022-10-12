from copy import deepcopy
from unittest import TestCase
from unittest.mock import MagicMock

import numpy as np

from sbp_env import visualiser
from sbp_env.env import Env
from sbp_env.planners.rrdtPlanner import RRdTSampler, Node
from sbp_env.utils import planner_registry
from tests.common_vars import template_args, MockNumpyEquality


class TestRRdTSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args["sampler_data_pack"].sampler_class = RRdTSampler

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrdt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler
        # self.env.args.planner.run_once()

        self.env.cc.feasible = MagicMock(return_value=True)
        self.env.cc.visible = MagicMock(return_value=True)

    def test_get_next_pos(self):
        self.env.args.epsilon = 10000
        # set random seed for consistent results
        np.random.seed(0)

        self.sampler.args.goalBias = 0

        # test all results are different
        results = set()
        for i in range(100):
            results.add(Node(self.sampler.get_next_pos()[0]))
        self.assertEqual(len(results), 100)

    def test_get_random_choice(self):
        # test if probability affects the random choice
        for i in range(self.sampler.num_dtrees):
            probs = np.zeros(self.sampler.num_dtrees)
            probs[i] = 1
            self.sampler.p_manager.get_prob = MagicMock(return_value=probs)
            self.assertEqual(self.sampler.get_random_choice(), i)

    def test_random_walk(self):
        for i in range(self.sampler.num_dtrees):
            desire_direction = np.random.rand(self.sampler.args.num_dim)
            desire_direction /= np.linalg.norm(desire_direction)
            self.sampler.p_manager.particles[i].draw_sample = MagicMock(
                return_value=desire_direction
            )

            # test if probability affects the random choice
            ori_pos = self.sampler.p_manager.get_pos(i)
            new_pos = self.sampler.random_walk(i)

            # ensure that the returned position is in the correct direction that the
            # particle reported
            particle_returned_dir = new_pos - ori_pos
            particle_returned_dir /= np.linalg.norm(particle_returned_dir)
            self.assertEqual(
                MockNumpyEquality(particle_returned_dir, almost_equal=True),
                desire_direction,
            )
