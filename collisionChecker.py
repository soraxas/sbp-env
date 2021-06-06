from abc import ABC, abstractmethod

import numpy as np


class CollisionChecker(ABC):
    """ """

    @abstractmethod
    def get_dimension(self):
        """ """
        pass

    def visible(self, posA, posB):
        """

        :param posA: 
        :param posB: 

        """
        pass

    def feasible(self, p):
        """

        :param p: 

        """
        pass

    def get_image_shape(self):
        """ """
        return None


class ImgCollisionChecker(CollisionChecker):
    def __init__(self, img):
        from PIL import Image

        image = Image.open(img).convert("L")
        image = np.array(image)
        image = image / 255
        # white pixxel should now have value of 1
        image[image != 1.0] = 0

        # import matplotlib.pyplot as plt
        # plt.imshow(image)
        # plt.colorbar()

        # need to transpose because pygame has a difference coordinate system than matplotlib matrix
        self.img = image.T

    def get_image_shape(self):
        """ """
        return self.img.shape

    def get_dimension(self):
        """ """
        return 2

    def get_coor_before_collision(self, posA, posB):
        """

        :param posA: 
        :param posB: 

        """
        pixels = self.get_line(posA, posB)
        # check that all pixel are white (free space)
        endPos = posB
        for p in pixels:
            endPos = (p[0], p[1])
            if not self.feasible(p):
                break
        return endPos

    def visible(self, posA, posB):
        """

        :param posA: 
        :param posB: 

        """
        try:
            # get list of pixel between node A and B
            # pixels = lineGenerationAlgorithm(posA, posB)
            pixels = self.get_line(posA, posB)
            # check that all pixel are white (free space)
            for p in pixels:
                if not self.feasible(p):
                    return False
        except ValueError:
            return False
        return True

    def feasible(self, p):
        """check if point is white (which means free space)

        :param p: 

        """
        try:
            return self.img[tuple(map(int, p))] == 1
        except IndexError:
            return False

    @staticmethod
    def get_line(start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end

        :param start: 
        :param end: 

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
    """ """

    def __init__(self, xml, stats):
        self.stats = stats
        import klampt
        from klampt.plan import robotplanning
        from klampt.io import resource

        self.cc_feasible = 0
        self.cc_visible = 0

        world = klampt.WorldModel()
        world.readFile(xml)  # very cluttered
        robot = world.robot(0)

        # this is the CSpace that will be used.  Standard collision and joint limit constraints
        # will be checked
        space = robotplanning.makeSpace(world, robot, edgeCheckResolution=0.1)

        # fire up a visual editor to get some start and goal configurations
        qstart = robot.getConfig()
        qgoal = robot.getConfig()

        self.space = space
        self.robot = robot
        self.world = world

        import copy

        self.template_pos = copy.copy(qstart)
        self.template_pos[1:7] = [0] * 6

        self.qstart = self.translate_from_klampt(qstart)
        self.qgoal = self.translate_from_klampt(qgoal)

    def get_dimension(self):
        """ """
        return 6

    def get_dimension_limits(self):
        """ """
        return self.robot.getJointLimits()

    def translate_to_klampt(self, p):
        """

        :param p: 

        """
        assert len(p) == 6, p
        import copy

        new_pos = list(self.template_pos)
        new_pos[1:7] = p
        return new_pos

    def translate_from_klampt(self, p):
        """

        :param p: 

        """
        assert len(p) == 12, len(p)
        return p[1:7]

    def visible(self, a, b):
        """

        :param a: 
        :param b: 

        """
        a = self.translate_to_klampt(a)
        b = self.translate_to_klampt(b)
        # print(self.space.visible(a, b))
        self.stats.visible_cnt += 1
        return self.space.isVisible(a, b)

    def feasible(self, p, stats=False):
        """

        :param p: 
        :param stats:  (Default value = False)

        """
        p = self.translate_to_klampt(p)
        self.stats.feasible_cnt += 1
        return self.space.feasible(p)


import math


class RobotArm4dCollisionChecker(CollisionChecker):
    def __init__(self, img, map_mat=None):
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
            self.img = image

        else:
            self.img = (map_mat / map_mat.max()).astype(int)

        # need to transpose because pygame has a difference coordinate system than matplotlib matrix
        self.img = self.img.T

        # self.stick_robot_length_config = stick_robot_length_config
        self.stick_robot_length_config = [35, 35]

    def get_image_shape(self):
        """ """
        return self.img.shape

    def get_dimension(self):
        """ """
        return 4

    def get_coor_before_collision(self, posA, posB):
        """

        :param posA: 
        :param posB: 

        """
        pixels = self._get_line(posA, posB)
        # check that all pixel are white (free space)
        endPos = posB
        for p in pixels:
            endPos = (p[0], p[1])
            if not self.feasible(p):
                break
        return endPos

    @staticmethod
    def create_ranges(start, stop, N, endpoint=True):
        """From https://stackoverflow.com/questions/40624409/vectorized-numpy-linspace-for-multiple-start-and-stop-values

        :param start: 
        :param stop: 
        :param N: 
        :param endpoint:  (Default value = True)

        """
        if endpoint == 1:
            divisor = N - 1
        else:
            divisor = N
        steps = (1.0 / divisor) * (stop - start)
        return steps[:, None] * np.arange(N) + start[:, None]

    def _interpolate_configs(self, c1, c2):
        """Given two configs (x, y, r1, r2), return intepolate in-between

        :param c1: 
        :param c2: 

        """
        loc_interpolate = self._get_line(c1[:2], c2[:2])
        # print(np.array(loc_interpolate).T)
        if len(loc_interpolate) == 1:
            # to hot-fix because config interpolate must be >= 2
            loc_interpolate.append(loc_interpolate[0])

        rot_interpolate = self.create_ranges(c1[2:], c2[2:], N=len(loc_interpolate))

        combined = np.concatenate([np.array(loc_interpolate).T, rot_interpolate]).T

        return combined

    def visible(self, posA, posB):
        """

        :param posA: 
        :param posB: 

        """
        # get list of pixel between node A and B
        # pixels = lineGenerationAlgorithm(posA, posB)
        for p in self._interpolate_configs(posA, posB):
            if not self.feasible(p):
                return False
        return True

    def _pt_feasible(self, p):
        """check if point is white (which means free space) in 2d

        :param p: 

        """
        try:
            return self.img[tuple(map(int, p))] == 1
        except IndexError:
            return False

    def feasible(self, p):
        """check if configuration is fesible

        :param p: 

        """
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
        """

        :param pt1: 
        :param angle: 
        :param line_length: 

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

        :param start: 
        :param end: 

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