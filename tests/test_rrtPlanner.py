from copy import deepcopy
from unittest import TestCase
from unittest.mock import MagicMock, ANY

import numpy as np

from env import Env
from samplers.randomPolicySampler import RandomPolicySampler
from tests.common_vars import template_args, MockNumpyEquality
from utils import planner_registry
from utils.common import Node


class TestRRTPlanner(TestCase):
    def setUp(self) -> None:
        args = deepcopy(template_args)

        # setup to use the correct sampler
        args["sampler"] = RandomPolicySampler()

        # use some suitable planner
        args["planner_data_pack"] = planner_registry.PLANNERS["rrt"]

        self.env = Env(args)
        self.sampler = self.env.args.sampler
        self.planner = self.env.args.planner

        self.planner.args.radius = 1000
        # make it always be visible for testing
        self.planner.args.env.cc.feasible = MagicMock(return_value=True)
        self.planner.args.env.cc.visible = MagicMock(return_value=True)

    def test_add_newnode(self):
        # repeat the testing multiple times
        for i in range(10):
            node = Node(np.random.rand(2))
            self.planner.add_newnode(node)

            # assert that the position is correct
            self.assertTrue(
                np.isclose(
                    self.planner.poses[len(self.planner.nodes) - 1], node.pos
                ).all()
            )
            # assert that the added node is correct
            self.assertEqual(self.planner.nodes[-1], node)

    def test_run_once_success(self):
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
        mocked_report_success.assert_called_once_with(
            pos=MockNumpyEquality(pos1), nn=ANY, rand_pos=MockNumpyEquality(pos1)
        )
        self.assertEqual(MockNumpyEquality(pos1), self.planner.nodes[-1].pos)

    def test_run_once_failed(self):
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
        mocked_report_fail.assert_called_once_with(
            pos=MockNumpyEquality(pos1), free=False
        )
        mocked_report_success.assert_not_called()

    def test_choose_least_cost_parent(self):
        node1 = Node(np.random.rand(2))
        node2 = Node(np.random.rand(2))
        node3 = Node(np.random.rand(2))
        node4 = Node(np.random.rand(2))
        node1.cost = 1
        node2.cost = 10
        node3.cost = 50

        # ensure that the nn returned is as expected
        newnode, nn = self.planner.choose_least_cost_parent(
            newnode=node4, nodes=[node1, node2, node3]
        )
        self.assertEqual(nn, node1)

        # ensure that the nn returned is as expected
        newnode, nn = self.planner.choose_least_cost_parent(
            newnode=node2, nodes=[node1, node4, node3]
        )
        self.assertEqual(nn, node1)

    def test_rewire(self):
        node1 = Node(np.random.rand(2))
        node2 = Node(np.random.rand(2))
        node3 = Node(np.random.rand(2))
        node4 = Node(np.random.rand(2))
        node1.cost = 1
        node2.cost = 10
        node3.cost = 50

        self.planner.add_newnode(node1)
        self.planner.add_newnode(node2)
        self.planner.add_newnode(node3)
        self.planner.add_newnode(node4)

        poses = np.array([node1.pos, node2.pos, node3.pos, node4.pos])

        newnode, nn = self.planner.choose_least_cost_parent(
            node4, node1, nodes=self.planner.nodes, poses=poses,
        )
        # newnode, nn, nodes=self.nodes, skip_optimality=True)
        self.planner.add_newnode(node1)
        # rewire to see what the newly added node can do for us
        self.planner.rewire(node1, self.planner.nodes, poses=poses)

    def test_find_nearest_neighbour_idx(self):
        node1 = Node(np.array([0, 0]))
        node2 = Node(np.array([1, 1]))
        node3 = Node(np.array([2, 2]))
        node4 = Node(np.array([3, 3]))

        nodes = [node1, node2, node3, node4]
        poses = np.array([n.pos for n in nodes])

        for i, n in enumerate(nodes):
            self.assertEqual(
                self.planner.find_nearest_neighbour_idx(n.pos + 1e-2, poses), i
            )
