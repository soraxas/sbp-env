from unittest import TestCase

import numpy as np

from sbp_env.collisionChecker import RobotArm4dCollisionChecker
from sbp_env.utils.common import Stats, MagicDict
from tests.test_image_space_collision_checker import (
    mock_image_as_np,
    create_test_image,
    pt,
)


class TestRobotArm4dCollisionChecker(TestCase):
    def setUp(self) -> None:
        self.cc = RobotArm4dCollisionChecker(
            create_test_image(),
            stick_robot_length_config=[0.1, 0.1],
            stats=Stats(),
        )
        self.target = mock_image_as_np == 255
        self.target = self.target.astype(self.cc.image.dtype)

    def test_create_ranges(self):
        starts = np.array([1, 5, 3, 19])
        ends = np.array([-11, 54, 32, -219])
        multi_linspace = self.cc.create_ranges(starts, ends, 10)

        # assert lengths are equal
        self.assertEqual(multi_linspace.shape[1], 10)
        self.assertEqual(multi_linspace.shape[0], len(starts))

        # assert they starts and ends at the specified locations
        self.assertTrue(np.isclose(multi_linspace[:, 0], starts).all())
        self.assertTrue(np.isclose(multi_linspace[:, -1], ends).all())

    def test_interpolate_configs(self):
        starts = np.random.rand(4) * 10
        ends = np.random.rand(4) * 10
        out = self.cc._interpolate_configs(starts, ends)

        self.assertTrue(out.shape[0] >= 2)
        for i in range(out.shape[0]):
            self.assertEqual(len(out[i, :]), 4)

        # the first two configs are pixel location, which will always be int
        self.assertTrue(
            np.isclose(out[0, :2].astype(np.uint8), starts[:2].astype(np.uint8)).all()
        )
        # the last two configs are rotational joints which will be in reals
        self.assertTrue(np.isclose(out[0, 2:], starts[2:]).all())

        # the first two configs are pixel location, which will always be int
        self.assertTrue(
            np.isclose(out[-1, :2].astype(np.uint8), ends[:2].astype(np.uint8)).all()
        )
        # the last two configs are rotational joints which will be in reals
        self.assertTrue(np.isclose(out[-1, 2:], ends[2:]).all())

    def test_visible(self):
        self.assertTrue(self.cc.visible(pt(0.5, 2.5, 1, 2.5), pt(1.5, 3.5, 0, -1.5)))
        self.assertTrue(self.cc.visible(pt(0.5, 2.5, -1, 1), pt(0.5, 3.5, 2, 0.4)))

    def test_not_visible(self):
        self.assertFalse(self.cc.visible(pt(0.5, 2.5, 1, 2.5), pt(1.5, 5.5, 0, -1.5)))
        self.assertFalse(self.cc.visible(pt(0.5, 0.5, -1, 1), pt(3.5, 0.5, 2, 0.4)))

    def test_feasible(self):
        self.assertTrue(self.cc.feasible(pt(0.5, 2.5, 1, 2.5)))
        self.assertTrue(self.cc.feasible(pt(1.5, 3.5, 0, -1.5)))
        self.assertTrue(self.cc.feasible(pt(0.5, 3.5, 2, 0.4)))

    def test_not_feasible(self):
        self.assertFalse(self.cc.feasible((3.5, 0.5, 1, 1)))
        self.assertFalse(self.cc.feasible((-1.5, 3.5, 1, 1)))
        self.assertFalse(self.cc.feasible((1.78, 4.5, 1, 1)))
