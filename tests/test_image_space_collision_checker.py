from unittest import TestCase

import numpy as np

from collisionChecker import ImgCollisionChecker
from tests.common_vars import create_test_image, mock_image_as_np
from utils.common import Stats

eps = 1e-5


def pt(*args):
    # helper to construct numpy array
    return np.array(args)


def pts_pair(args1, args2):
    # helper to add and subtract eps to help distinguish the requested pts from the
    # boundary line
    pt1 = pt(*args1)
    pt2 = pt(*args2)
    return pt1 + eps, pt2 - eps


class TestImgCollisionChecker(TestCase):
    def setUp(self) -> None:
        self.cc = ImgCollisionChecker(create_test_image(), stats=Stats())
        self.target = mock_image_as_np == 255
        self.target = self.target.astype(self.cc.image.dtype)

    def test_conversion(self):
        self.assertTrue(np.isclose(self.cc.image, self.target).all())

    def test_get_image_shape(self):
        self.assertEqual(self.cc.get_image_shape(), self.target.T.shape)

    def test_get_dimension(self):
        self.assertEqual(self.cc.get_dimension(), 2)

    def test_get_coor_before_collision(self):
        self.assertEqual(
            self.cc.get_coor_before_collision(*pts_pair([0, 2], [2, 6])), (1, 4)
        )
        self.assertEqual(
            self.cc.get_coor_before_collision(*pts_pair([0, 2], [2, 4])), (1, 3)
        )

    def test_visible(self):
        self.assertTrue(self.cc.visible(*pts_pair([0, 2], [2, 4])))
        self.assertTrue(self.cc.visible(*pts_pair([0, 2], [0, 4])))

    def test_not_visible(self):
        self.assertFalse(self.cc.visible(*pts_pair([0, 2], [2, 6])))
        self.assertFalse(self.cc.visible(*pts_pair([0, 0], [4, 0])))

    def test_feasible(self):
        self.assertTrue(self.cc.feasible((1.5, 0.5)))
        self.assertTrue(self.cc.feasible((1.5, 3.5)))
        self.assertTrue(self.cc.feasible((1.78, 2.5)))

    def test_not_feasible(self):
        self.assertFalse(self.cc.feasible((3.5, 0.5)))
        self.assertFalse(self.cc.feasible((-1.5, 3.5)))
        self.assertFalse(self.cc.feasible((1.78, 4.5)))
