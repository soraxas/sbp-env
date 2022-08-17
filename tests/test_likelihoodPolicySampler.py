from copy import deepcopy
from unittest import TestCase

import numpy as np

import visualiser
from env import Env
from samplers.likelihoodPolicySampler import LikelihoodPolicySampler
from tests.common_vars import template_args
from utils import planner_registry
from utils.common import Node


class TestLikelihoodPolicySampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args["prob_block_size"] = 10
        args["sampler_data_pack"].sampler_class = LikelihoodPolicySampler

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler

    def test_likelihood_increases(self):
        pt, report_success, report_fail = self.sampler.get_next_pos()

        target_pt = np.array([10, 10])
        nn = Node(np.array([20, 20]))
        rand_pos = np.array([5, 5])

        prev_val_1 = None
        prev_val_2 = None
        prev_val_3 = None

        for i in range(20):
            report_success(
                pos=target_pt,
                nn=nn,
                rand_pos=rand_pos,
            )

            # val_1 comes from the target point (probability should increases)
            val_1 = self.sampler.prob_vector[
                tuple((target_pt / self.sampler.PROB_BLOCK_SIZE).astype(int))
            ]
            if prev_val_1 is not None:
                # the likelihood should increases
                self.assertTrue(val_1 > prev_val_1)
            prev_val_1 = val_1

            # val_2 comes from the nearest neighbour point (probability should
            # increases)
            val_2 = self.sampler.prob_vector[
                tuple((nn.pos / self.sampler.PROB_BLOCK_SIZE).astype(int))
            ]
            if prev_val_2 is not None:
                # the likelihood should increases
                self.assertTrue(val_2 > prev_val_2)
            prev_val_2 = val_2

            # val_3 comes from the desire rand pos (but hasn't reached yet,
            # should this should not be affected)
            val_3 = self.sampler.prob_vector[
                tuple((rand_pos / self.sampler.PROB_BLOCK_SIZE).astype(int))
            ]
            if prev_val_3 is not None:
                # the likelihood should remains the same
                self.assertTrue(val_3 == prev_val_3)
            prev_val_3 = val_3
