from copy import deepcopy
from unittest import TestCase

import numpy as np

from sbp_env import visualiser
from sbp_env.env import Env
from sbp_env.planners.rrdtPlanner import Node
from sbp_env.samplers.prmSampler import PRMSampler
from sbp_env.utils import planner_registry
from tests.common_vars import template_args


class TestPRMSampler(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args["sampler_data_pack"].sampler_class = PRMSampler

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["prm"]

        self.env = Env(args)
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
