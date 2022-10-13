from abc import ABC, abstractmethod

from typing import Callable, Optional

import numpy as np
import math

from . import collisionChecker
from .utils.common import PlanningOptions, cached_property
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
    def __init__(self, cc: collisionChecker.CollisionChecker):
        self.cc = cc

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

    @abstractmethod
    def get_dimension(self) -> int:
        """Get the current dimensionality of this space

        :return: the dimensionality
        """
        raise NotImplementedError("Must derive from this class")


class ImageEngine(Engine):
    def __init__(self, args: PlanningOptions, image: str):
        super().__init__(
            collisionChecker.ImgCollisionChecker(
                image,
            )
        )
        VisualiserSwitcher.choose_visualiser("pygame")

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([0, 0])

    @cached_property
    def upper(self) -> np.ndarray:
        """Get the image shape of the planning problem.

        :return: the shape of the planning problem
        """
        return np.array(self.cc._img.shape)

    def get_dimension(self) -> int:
        """Get the current dimensionality of this space

        :return: the dimensionality
        """
        return 2


class RobotArmEngine(Engine):
    def __init__(self, args: PlanningOptions, image: str):
        super().__init__(
            collisionChecker.RobotArm4dCollisionChecker(
                image,
                stick_robot_length_config=args.rover_arm_robot_lengths,
            )
        )
        VisualiserSwitcher.choose_visualiser("pygame")

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([0, 0, -np.pi, -np.pi])

    @cached_property
    def upper(self) -> np.ndarray:
        return np.array([self.cc._img.shape[0], self.cc._img.shape[1], np.pi, np.pi])

    def get_dimension(self) -> int:
        """Get the current dimensionality of this space

        :return: the dimensionality
        """
        return 4


class KlamptEngine(Engine):
    def __init__(self, args: PlanningOptions, xml_file: str):
        super().__init__(
            collisionChecker.KlamptCollisionChecker(
                xml_file,
            )
        )
        VisualiserSwitcher.choose_visualiser("klampt")

    @cached_property
    def lower(self) -> np.ndarray:
        return np.array([-np.pi] * self.get_dimension())

    @cached_property
    def upper(self) -> np.ndarray:
        return np.array([np.pi] * self.get_dimension())

    def get_dimension(self) -> int:
        """Get the current dimensionality of this space

        :return: the dimensionality
        """
        return 6


class BlackBoxEngine(Engine):
    def __init__(
        self,
        collision_checking_functor: collisionChecker.BlackBoxCollisionChecker.CCType,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
        dist_functor: Optional[Callable[[np.ndarray, np.ndarray], float]] = None,
        cc_epsilon: float = 0.1,
    ):
        super().__init__(
            collisionChecker.BlackBoxCollisionChecker(
                collision_checking_functor=collision_checking_functor,
                cc_epsilon=cc_epsilon,
            )
        )
        lower_limits = np.array(lower_limits)
        upper_limits = np.array(upper_limits)
        if lower_limits.shape[0] != upper_limits.shape[0]:
            raise ValueError("Lower and upper limit mismatch!")
        self.__dim = lower_limits.shape[0]
        self.__lower_limits = lower_limits
        self.__upper_limits = upper_limits
        if dist_functor:
            self.dist = dist_functor

        if self.__dim == 2:
            VisualiserSwitcher.choose_visualiser("blackbox")

    @cached_property
    def lower(self) -> np.ndarray:
        return self.__lower_limits

    @cached_property
    def upper(self) -> np.ndarray:
        return self.__upper_limits

    def get_dimension(self) -> int:
        """Get the current dimensionality of this space

        :return: the dimensionality
        """
        return self.__dim
