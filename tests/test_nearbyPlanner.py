from copy import deepcopy
from unittest.mock import MagicMock

from sbp_env import visualiser
from sbp_env.env import Env
from sbp_env.samplers.nearbyPolicySampler import NearbyPolicySampler
from sbp_env.utils import planner_registry
from tests.common_vars import template_args
from tests.test_rrtPlanner import TestRRTPlanner


# reuse some of the test from RRTPlanner
class TestNearbyPolicyPlanner(TestRRTPlanner):
    def setUp(self) -> None:
        args = deepcopy(template_args)
        visualiser.VisualiserSwitcher.choose_visualiser("base")

        # setup to use the correct sampler
        args.sampler = NearbyPolicySampler(prob_block_size=10)

        # use some suitable planner
        args.planner_data_pack = planner_registry.PLANNERS["rrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

        self.planner.args.radius = 1000
        # make it always be visible for testing
        self.planner.args.engine.cc.feasible = MagicMock(return_value=True)
        self.planner.args.engine.cc.visible = MagicMock(return_value=True)
