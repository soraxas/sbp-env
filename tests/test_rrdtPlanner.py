from copy import deepcopy
from functools import partialmethod
from unittest.mock import MagicMock, ANY

import numpy as np
from tqdm import tqdm

from env import Env
from planners.rrdtPlanner import RRdTSampler
from tests.common_vars import template_args, MockNumpyEquality
from tests.test_rrtPlanner import TestRRTPlanner
from utils import planner_registry

# the following disable printing from tqdm module during testing
tqdm.__init__ = partialmethod(tqdm.__init__, disable=True)


class TestRRdTPlanner(TestRRTPlanner):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = RRdTSampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrdt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

        self.planner.args.radius = 1000
        self.planner.args.epsilon = 1000
        # make it always be visible for testing
        self.planner.args.env.cc.feasible = MagicMock(return_value=True)
        self.planner.args.env.cc.visible = MagicMock(return_value=True)

    @staticmethod
    def flush_rrdt_sampler_pending_restarts(sampler):
        # flush all particle that are pending to be restarts
        # this is needed when rrdt sampler's get_next_pos is mocked, as the following
        # function is a side-effect of `get_next_pos`

        # force the flag to be false so it won't keep trying to restarts
        sampler.restart_when_merge = False
        while not sampler.restart_all_pending_local_samplers():
            pass
        sampler.restart_when_merge = True

    def test_run_once_success(self):
        pos1 = np.random.rand(2)
        self.planner.args.env.cc.visible = MagicMock(return_value=True)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(
                pos1,
                self.planner._disjointed_trees[-1],
                None,
                mocked_report_success,
                mocked_report_fail,
            ),
        )
        self.flush_rrdt_sampler_pending_restarts(self.sampler)
        num_nodes_before = len(self.planner.nodes)
        self.planner.run_once()
        self.assertTrue(len(self.planner.nodes) >= num_nodes_before + 1)

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_called_once_with(
            pos=MockNumpyEquality(pos1), newnode=ANY,
        )
        self.assertTrue(any(np.isclose(pos1, n.pos).all() for n in self.planner.nodes))

    def test_run_once_failed(self):
        pos1 = np.random.rand(2) * 10
        self.planner.args.env.cc.visible = MagicMock(return_value=False)

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(
            return_value=(
                pos1,
                self.planner._disjointed_trees[0],
                None,
                mocked_report_success,
                mocked_report_fail,
            )
        )
        self.planner.run_once()

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_called_once_with(pos=pos1, free=False)
        mocked_report_success.assert_not_called()
        self.assertEqual(len(self.planner.nodes), 1)

    def test_run_once_return_None(self):
        # check that if get_next_pos returned None, the planner will assume that
        # the sampler itself had added node while restarting the particles
        self.planner.args.env.cc.visible = MagicMock()
        self.planner.args.env.cc.feasible = MagicMock()

        mocked_report_success = MagicMock()
        mocked_report_fail = MagicMock()

        # mock the returning sampling point
        self.sampler.get_next_pos = MagicMock(return_value=None)
        self.planner.run_once()

        self.sampler.get_next_pos.assert_called_once_with()
        mocked_report_fail.assert_not_called()
        mocked_report_success.assert_not_called()
        self.planner.args.env.cc.visible.assert_not_called()
        self.planner.args.env.cc.feasible.assert_not_called()
