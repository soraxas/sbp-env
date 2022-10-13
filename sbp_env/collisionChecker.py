import math
import typing
from abc import ABC, abstractmethod

import numpy as np

from .utils.common import Stats, PlanningOptions


class CollisionChecker(ABC):
    """Abstract collision checker"""

    def __init__(self):
        if not Stats.has_instance():
            Stats.build_instance()
        self.stats = Stats.get_instance()

    def visible(self, pos1: np.ndarray, pos2: np.ndarray):
        r"""Check if the straight line connection between pos1 and pos2 is in
        :math:`C_\text{free}`. Internally it might calls :meth:`feasible`.

        :param pos1: the starting configuration of the line
        :param pos2: the target configuration of th eline

        """
        raise NotImplementedError("Must derive from this class")

    def feasible(self, p: np.ndarray):
        r"""Check if the configuration is in free-space, i.e.,
        :math:`q \in C_\text{free}`

        :param p: configuration to check

        """
        raise NotImplementedError("Must derive from this class")


class BlackBoxCollisionChecker(CollisionChecker):
    """Abstract collision checker"""

    CCType = typing.Callable[[np.ndarray], bool]

    def __init__(
        self,
        collision_checking_functor: CCType,
        cc_epsilon: float,
    ):
        super().__init__()
        self.__cc_functor = collision_checking_functor
        self.__cc_epsilon = cc_epsilon

    @property
    def epsilon(self) -> float:
        return self.__cc_epsilon

    def visible(self, pos1: np.ndarray, pos2: np.ndarray):
        r"""Check if the straight line connection between pos1 and pos2 is in
        :math:`C_\text{free}`. Internally it might calls :meth:`feasible`.

        :param pos1: the starting configuration of the line
        :param pos2: the target configuration of th eline

        """
        return all(
            self.feasible(pos1 + t)
            for t in np.arange(
                0, np.linalg.norm(pos2 - pos1), self.__cc_epsilon, dtype=float
            )
        )

    def feasible(self, p: np.ndarray):
        r"""Check if the configuration is in free-space, i.e.,
        :math:`q \in C_\text{free}`

        :param p: configuration to check

        """
        return self.__cc_functor(p)


class ImgCollisionChecker(CollisionChecker):
    """
    2D Image Space simulator engine.py
    """

    def __init__(
        self,
        img: str,
    ):
        """
        :param img: a file-like object (e.g. a filename) for the image as the
            environment that the planning operates in
        :param stats: the Stats object to keep track of stats
        """
        super().__init__()
        from PIL import Image

        self.image_fname = img
        image = Image.open(img).convert("L")
        image = np.array(image)
        image = image / 255
        # white pixxel should now have value of 1
        image[image != 1.0] = 0

        # need to transpose because pygame has a difference coordinate system than matplotlib matrix
        self._img = image.T

    @property
    def image(self) -> np.ndarray:
        """The image that represents the planning problem

        :return: the input image
        """
        return self._img.T

    def get_coor_before_collision(self, pos1, pos2):
        """Get the list of coordinate before the collision

        :param pos1: first configuration
        :param pos2: second configuration

        """
        pixels = self.get_line(pos1, pos2)
        # check that all pixel are white (free space)
        endPos = pos2
        for p in pixels:
            endPos = (p[0], p[1])
            if not self.feasible(p):
                break
        return endPos

    def visible(self, pos1, pos2):
        self.stats.visible_cnt += 1
        try:
            # get list of pixel between node A and B
            pixels = self.get_line(pos1, pos2)
            # check that all pixel are white (free space)
            for p in pixels:
                if not self.feasible(p, save_stats=False):
                    return False
        except ValueError:
            return False
        return True

    def feasible(self, p, save_stats=True):
        if save_stats:
            self.stats.feasible_cnt += 1
        try:
            return self._img[tuple(map(int, p))] == 1
        except IndexError:
            return False

    @staticmethod
    def get_line(start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end

        :param start: the starting pixel coordinate
        :param end: the ending pixel coordinate

        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = map(int, start)
        x2, y2 = map(int, end)
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


class KlamptCollisionChecker(CollisionChecker):
    """A wrapper around Klampt's 3D simulator"""

    def __init__(
        self,
        xml: str,
    ):
        """
        :param xml: the xml filename for Klampt to read the world settings
        :param stats: the Stats object to keep track of stats
        """
        super().__init__()
        import klampt
        from klampt.plan import robotplanning

        world = klampt.WorldModel()
        world.readFile(xml)  # very cluttered
        robot = world.robot(0)

        # this is the CSpace that will be used.
        # Standard collision and joint limit constraints will be checked
        space = robotplanning.makeSpace(world, robot, edgeCheckResolution=0.1)

        self.space = space
        self.robot = robot
        self.world = world

        import copy

        self.template_pos = copy.copy(robot.getConfig())
        self.template_pos[1:7] = [0] * 6

    def get_dimension_limits(self):
        return self.space.bound

    def translate_to_klampt(self, p):
        """Translate the given configuration to klampt's configuration

        :param p: configuration to translate

        """
        # assert len(p) == self.get_dimension(), p
        # add a dummy end configuration that denotes the gripper's angle
        return list(p) + [0]

    def translate_from_klampt(self, p):
        """Translate the given klampt's configuration to our protocol

        :param p: configuration to translate

        """
        if len(p) == 12:
            # this is what klampt's robot interface return, mixed with arm joints and
            # gripper angles
            return p[1:7]
        elif len(p) == 7:
            # this is what klampt's space interface return, mixed with arm joints and
            # gripper rotation (single value)
            return p[:6]
        raise ValueError(f"Unknown length {p}")

    def get_eef_world_pos(self, config):
        _config = self.robot.getConfig()
        _config[1:7] = list(config)[:6]
        self.robot.setConfig(_config)
        link = self.robot.link(11)  # the eef
        pos = link.getWorldPosition([0, 0, 0])
        return pos

    def visible(self, a, b):
        a = self.translate_to_klampt(a)
        b = self.translate_to_klampt(b)
        self.stats.visible_cnt += 1
        return self.space.isVisible(a, b)

    def feasible(self, p, save_stats=True):
        p = self.translate_to_klampt(p)
        self.stats.feasible_cnt += 1
        return self.space.isFeasible(p)


class RobotArm4dCollisionChecker(CollisionChecker):
    """
    4D robot arm simulator engine.py that operates in the Image Space
    """

    def __init__(
        self,
        img: str,
        map_mat: typing.Optional[np.ndarray] = None,
        stick_robot_length_config: typing.Optional[typing.Sequence[float]] = None,
    ):
        """

        :param img: a file-like object (e.g. a filename) for the image as the
            environment that the planning operates
        :param map_mat: an image that, if given, will ignore the `img` argument and
            uses `map_mat` directly as the map
        :param stick_robot_length_config: a list of numbers that represents the
            length of the stick robotic arm
        :param stats: the Stats object to keep track of stats
        """
        super().__init__()
        self.image_fname = img
        if map_mat is None:
            from PIL import Image

            image = Image.open(img).convert("L")
            image = np.array(image)
            image = image / 255
            # white pixxel should now have value of 1
            image[image != 1.0] = 0

            # import matplotlib.pyplot as plt
            # plt.imshow(image)
            # plt.colorbar()
            self._img = image

        else:
            self._img = (map_mat / map_mat.max()).astype(int)

        if stick_robot_length_config is not None:
            self.stick_robot_length_config = stick_robot_length_config

        # need to transpose because pygame has a difference coordinate system than matplotlib matrix
        self._img = self._img.T

    @property
    def image(self):
        return self._img.T

    @staticmethod
    def create_ranges(
        start: np.ndarray, stop: np.ndarray, N: int, endpoint: bool = True
    ):
        """Create a batch of linspace.

        :param start: an array of starting values
        :param stop: an array of stopping values
        :param N: the size of linspace
        :param endpoint: whether to include end point or not

        """
        # From https://stackoverflow.com/questions/40624409/vectorized-numpy-linspace-for-multiple-start-and-stop-values
        if endpoint == 1:
            divisor = N - 1
        else:
            divisor = N
        steps = (1.0 / divisor) * (stop - start)
        return steps[:, None] * np.arange(N) + start[:, None]

    def _interpolate_configs(self, c1, c2):
        """Given two configs (x, y, r1, r2), return interpolate in-between

        :param c1: the first configuration
        :param c2: the second configuration

        """
        loc_interpolate = self._get_line(c1[:2], c2[:2])
        # print(np.array(loc_interpolate).T)
        if len(loc_interpolate) == 1:
            # because config interpolate must be >= 2
            loc_interpolate.append(loc_interpolate[0])

        rot_interpolate = self.create_ranges(c1[2:], c2[2:], N=len(loc_interpolate))

        combined = np.concatenate([np.array(loc_interpolate).T, rot_interpolate]).T

        return combined

    def visible(self, pos1, pos2):
        # get list of pixel between node A and B
        self.stats.visible_cnt += 1
        for p in self._interpolate_configs(pos1, pos2):
            if not self.feasible(p, save_stats=False):
                return False
        return True

    def _pt_feasible(self, p):
        """check if point is white (which means free space) in 2d

        :param p: the configuration to check

        """
        try:
            return self._img[tuple(map(int, p))] == 1
        except IndexError:
            return False

    def feasible(self, p, save_stats=True):
        if save_stats:
            self.stats.feasible_cnt += 1
        pt1 = p[:2]
        pt2 = self.get_pt_from_angle_and_length(
            pt1, p[2], self.stick_robot_length_config[0]
        )

        # this stick should be free
        for __p in self._get_line(pt1, pt2):
            if not self._pt_feasible(__p):
                return False

        pt3 = self.get_pt_from_angle_and_length(
            pt2, p[3], self.stick_robot_length_config[1]
        )

        # this stick should be free
        for __p in self._get_line(pt2, pt3):
            if not self._pt_feasible(__p):
                return False

        return True

    @staticmethod
    def get_pt_from_angle_and_length(pt1, angle, line_length):
        """Obtain point 2 based of the given settings

        :param pt1: coordinate of pt1
        :param angle: the current angle
        :param line_length: the length of the arm

        """
        pt2 = (
            pt1[0] + line_length * math.cos(angle),
            pt1[1] + line_length * math.sin(angle),
        )
        return pt2

    @staticmethod
    def _get_line(start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end

        :param start: the start configuration
        :param end: the end configuration

        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = map(int, start)
        x2, y2 = map(int, end)
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else [x, y]
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points
