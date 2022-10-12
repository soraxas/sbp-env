from abc import ABC
from functools import cached_property

import numpy as np
import math

from . import collisionChecker
from .utils.common import MagicDict
from .visualiser import VisualiserSwitcher


def euclidean_dist(p1: np.ndarray, p2: np.ndarray):
    """Return the Euclidean distance between p1 and p2

    :param p1: first configuration :math:`q_1`
    :param p2: second configuration :math:`q_2`

    """
    # THIS IS MUCH SLOWER for small array
    # return np.linalg.norm(p1 - p2)

    p = p1 - p2
    return math.sqrt(p[0] ** 2 + p[1] ** 2)


class Engine(ABC):
    def __init__(self, args: MagicDict):
        self.args = args
        self.cc = None

    def get_dimension(self):
        return self.cc.get_dimension()

    def dist(self, *args):
        return euclidean_dist(*args)

    @cached_property
    def lower(self) -> np.ndarray:
        raise NotImplementedError()

    @cached_property
    def upper(self) -> np.ndarray:
        raise NotImplementedError()

    #
    # @cached_property
    # def dim(self) -> int:
    #     raise NotImplementedError()

    def transform(self, qs: np.ndarray) -> np.ndarray:
        """
        Given qs within the range of 0 to 1, project it to the full range of the bounds.

        :param qs:
        :return:
        """
        return qs * (self.upper - self.lower) + self.lower


class ImageEngine(Engine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cc = collisionChecker.ImgCollisionChecker(
            self.args.image, stats=self.args.stats, args=self.args
        )
        VisualiserSwitcher.choose_visualiser("pygame")

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([0, 0])

    @cached_property
    def upper(self) -> np.ndarray:
        return np.array(self.cc.get_image_shape())


class RobotArmEngine(Engine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cc = collisionChecker.RobotArm4dCollisionChecker(
            self.args.image, stats=self.args.stats, args=self.args
        )
        VisualiserSwitcher.choose_visualiser("pygame")
        self.image_shape = self.cc.get_image_shape()

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([0, 0, -np.pi, -np.pi])

    @cached_property
    def upper(self) -> np.ndarray:
        return np.array([self.image_shape[0], self.image_shape[1], np.pi, np.pi])


class KlamptEngine(Engine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cc = collisionChecker.KlamptCollisionChecker(
            self.args.image, stats=self.args.stats, args=self.args
        )
        VisualiserSwitcher.choose_visualiser("klampt")

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([-np.pi] * self.get_dimension())

    @cached_property
    def upper(self) -> np.ndarray:
        return np.array([np.pi] * self.get_dimension())
