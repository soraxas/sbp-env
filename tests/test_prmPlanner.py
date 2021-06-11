from copy import deepcopy
from unittest.mock import MagicMock

import numpy as np

from env import Env
from samplers.prmSampler import PRMSampler
from tests.common_vars import template_args, MockNumpyEquality
from tests.test_rrtPlanner import TestRRTPlanner
from utils import planner_registry


# reuse some of the test from RRTPlanner
class PRMPlanner(TestRRTPlanner):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = PRMSampler(prob_block_size=10)

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["prm"]

        self.env = Env(**args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

        self.planner.args.radius = 1000
        # make it always be visible for testing
        self.planner.args.env.cc.feasible = MagicMock(return_value=True)
        self.planner.args.env.cc.visible = MagicMock(return_value=True)

    def test_run_once_success(self):
        # PRM won't knows whether the sampled point is connectable or not before
        # graph construction
        pos1 = np.array([4, 2])
        self.planner.args.env.cc.visible = MagicMock(return_value=True)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(pos1, mocked_report_success, mocked_report_fail)
        )
        self.planner.run_once()

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_not_called()
        self.assertEqual(MockNumpyEquality(pos1), self.planner.nodes[-1].pos)

    def test_run_once_failed(self):
        # PRM won't knows whether the sampled point is connectable or not before
        # graph construction
        pos1 = np.array([4, 2])
        self.planner.args.env.cc.visible = MagicMock(return_value=False)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(pos1, mocked_report_success, mocked_report_fail)
        )
        self.planner.run_once()

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_not_called()
        self.assertEqual(MockNumpyEquality(pos1), self.planner.nodes[-1].pos)

    def test_build_graph(self):
        self.planner.args.env.cc.visible = MagicMock(return_value=True)
        self.planner.args.epsilon = 10000

        for i in range(10):
            self.planner.run_once()

        # construct the G = (V, E) graph
        self.planner.build_graph()
        self.assertEqual(len(self.planner.nodes), 10 + 1)  # +1 is the starting node
        print(self.planner.get_solution())
        self.assertTrue(self.planner.get_solution() < float("inf"))

    def test_build_graph_failed(self):
        self.planner.args.env.cc.visible = MagicMock(return_value=False)
        self.planner.args.epsilon = 10000

        for i in range(10):
            self.planner.run_once()

        # construct the G = (V, E) graph
        self.planner.build_graph()
        self.assertEqual(len(self.planner.nodes), 10 + 1)  # +1 is the starting node
        self.assertTrue(self.planner.get_solution() == float("inf"))
