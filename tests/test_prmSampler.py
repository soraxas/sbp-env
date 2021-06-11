from copy import deepcopy
from unittest import TestCase

import numpy as np

from env import Env
from planners.rrdtPlanner import Node
from samplers.prmSampler import PRMSampler
from tests.common_vars import template_args
from utils import planner_registry


class TestPRMSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = PRMSampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["prm"]

        self.env = Env(**args)
        self.sampler = self.env.args.sampler

    def test_init(self):
        # method that does not exists
        with self.assertRaises(ValueError):
            PRMSampler(random_method="foo bar")
        # with supported method
        PRMSampler(random_method="fast")

    def test_get_next_pos(self):
        # test that by default there should be no goal bias
        self.assertEqual(self.sampler.args.goalBias, 0)

        np.random.seed(0)

        # test all results are different
        results = set()
        for i in range(100):
            results.add(Node(self.sampler.get_next_pos()[0]))
        self.assertEqual(len(results), 100)
