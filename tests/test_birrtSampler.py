from copy import deepcopy
from unittest import TestCase

import numpy as np

import visualiser
from env import Env
from samplers.birrtSampler import BiRRTSampler
from tests.common_vars import template_args
from utils import planner_registry


class TestBiRRTSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args["sampler"] = BiRRTSampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["birrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler

    def test_init(self):
        # method that does not exists
        with self.assertRaises(ValueError):
            sampler = BiRRTSampler(random_method="foo bar")
            sampler.init(**self.env.args)
        # with supported method
        sampler = BiRRTSampler(random_method="fast")
        sampler.init(**self.env.args)

    def test_get_next_pos(self):
        # assert it go in-between start and goal post for goalBias

        self.sampler.args.planner.goal_tree_turn = True
        self.sampler.args.goalBias = 1

        for i in range(10):
            # goal bias will alternative in-between start and goal tree
            if i % 2 == 0:
                self.assertTrue(
                    np.isclose(
                        self.sampler.get_next_pos()[0], self.sampler.args.start_pt
                    ).all()
                )
            else:
                self.assertTrue(
                    np.isclose(
                        self.sampler.get_next_pos()[0], self.sampler.args.goal_pt
                    ).all()
                )
            self.sampler.args.planner.goal_tree_turn = (
                not self.sampler.args.planner.goal_tree_turn
            )
