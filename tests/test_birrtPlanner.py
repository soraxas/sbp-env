from copy import deepcopy
from unittest.mock import MagicMock, ANY

import numpy as np

from sbp_env import visualiser
from sbp_env.env import Env
from sbp_env.samplers.birrtSampler import BiRRTSampler
from sbp_env.utils import planner_registry
from tests.common_vars import template_args, MockNumpyEquality
from tests.test_rrtPlanner import TestRRTPlanner


# reuse some of the test from RRTPlanner
class TestBiRRTPlanner(TestRRTPlanner):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args.sampler = BiRRTSampler()

        # use some suitable planner
        args.planner_data_pack = planner_registry.PLANNERS["birrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

        self.planner.args.radius = 1000
        # make it always be visible for testing
        self.planner.args.engine.cc.feasible = MagicMock(return_value=True)
        self.planner.args.engine.cc.visible = MagicMock(return_value=True)

    def test_run_once_success(self):
        pos1 = np.array([101, 102])
        pos2 = np.array([40, 20])
        self.planner.args.engine.cc.visible = MagicMock(return_value=True)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(pos1, mocked_report_success, mocked_report_fail)
        )

        # this will be run on root tree
        self.planner.run_once()
        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_called_once_with(
            pos=ANY, nn=ANY, rand_pos=MockNumpyEquality(pos1)
        )
        self.assertEqual(MockNumpyEquality(pos1), self.planner.nodes[-1].pos)

        # reset mocks
        mocked_report_success.reset_mock()
        mocked_report_fail.reset_mock()
        self.sampler.get_next_pos = MagicMock(
            return_value=(pos2, mocked_report_success, mocked_report_fail)
        )

        # this will be run on goal root tree
        self.planner.run_once()
        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_called_once_with(
            pos=ANY, nn=ANY, rand_pos=MockNumpyEquality(pos2)
        )

    def test_run_once_failed(self):
        pos1 = np.array([101, 102])
        self.planner.args.engine.cc.visible = MagicMock(return_value=False)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(pos1, mocked_report_success, mocked_report_fail)
        )
        self.planner.run_once()

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_called_once_with(
            pos=MockNumpyEquality(pos1), free=False
        )
        mocked_report_success.assert_not_called()
