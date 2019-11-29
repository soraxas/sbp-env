import math
from abc import ABC, abstractmethod

import numpy as np
from helpers import Colour


class CollisionChecker(ABC):

    @abstractmethod
    def get_dimension(self):
        pass

    def visible(self, posA, posB):
        pass

    def feasible(self, p):
        pass


class ImgCollisionChecker(CollisionChecker):

    def __init__(self, img):
        """Short summary.

        Parameters
        ----------
        img : str
            Filename of the image where white pixel represents free space.
        """
        import matplotlib.image as mimg
        from PIL import Image
        image = Image.open(img).convert('L')
        image = np.array(image)
        image = image / 255
        # white pixxel should now have value of 1
        image[image != 1.0] = 0

        # import matplotlib.pyplot as plt
        # plt.imshow(image)
        # plt.colorbar()

        # need to transpose because pygame has a difference coordinate system than matplotlib matrix
        self.img = image.T

    def get_dimension(self):
        return self.img.shape

    def get_coor_before_collision(self, posA, posB):
        pixels = self._get_line(posA, posB)
        # check that all pixel are white (free space)
        endPos = posB
        for p in pixels:
            endPos = (p[0], p[1])
            if not self.feasible(p):
                break
        return endPos

    def visible(self, posA, posB):
        # get list of pixel between node A and B
        # pixels = lineGenerationAlgorithm(posA, posB)
        pixels = self._get_line(posA, posB)
        # check that all pixel are white (free space)
        for p in pixels:
            if not self.feasible(p):
                return False
        return True

    def feasible(self, p):
        """check if point is white (which means free space)"""
        try:
            return self.img[tuple(map(int, p))] == 1
        except IndexError:
            return False

    @staticmethod
    def _get_line(start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end

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
