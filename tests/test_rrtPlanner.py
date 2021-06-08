from copy import deepcopy
from unittest import TestCase

import numpy as np

from env import Env
from samplers.randomPolicySampler import RandomPolicySampler
from tests.common_vars import template_args
from utils import planner_registry
from utils.common import Node


class TestRRTPlanner(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = RandomPolicySampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrt"]

        self.env = Env(**args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

    #
    # def test_run_once(self):
    #     self.fail()

    def test_add_newnode(self):
        for i in range(10):
            node = Node(np.random.rand(2))
            self.planner.add_newnode(node)
            self.assertTrue(
                np.isclose(
                    self.planner.poses[len(self.planner.nodes) - 1], node.pos
                ).all()
            )
            self.assertEqual(self.planner.nodes[-1], node)

    # def test_choose_least_cost_parent(self):
    #     self.fail()
    #
    # def test_rewire(self):
    #     self.fail()
    #
    # def test_find_nearest_neighbour_idx(self):
    #     self.fail()
