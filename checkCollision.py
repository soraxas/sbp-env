import math

import numpy as np


def dist(p1, p2):
    # THIS IS MUCH SLOWER for small array
    # return np.linalg.norm(p1 - p2)
    p = p1-p2;
    return math.sqrt(p[0]**2 + p[1]**2)

def dist_all(p, pts):
    return np.linalg.norm(pts - p, axis=1)
    # p = p1-p2;
    # return math.sqrt(p[0]**2 + p[1]**2)

def dist_all2(node, nodes):
    shortest = node.pos - nodes[0].pos
    shortest = math.sqrt(shortest[0]**2 + shortest[1]**2)
    idx = 0
    for i in range(len(nodes)):
        shortest_tmp = node.pos - nodes[i].pos
        shortest_tmp = math.sqrt(shortest_tmp[0]**2 + shortest_tmp[1]**2)
        if shortest_tmp < shortest:
            shortest = shortest_tmp
            idx = i
    return i

def get_line(start, end):
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

# def lineGenerationAlgorithm(p1, p2):
#     pixels = []
#     # edge case of p1 == p2
#     if np.array_equal(p1,p2):
#         return []
#     # given 2 points, return a list that contain all pixel within the two point
#     dx = p2[0] - p1[0]
#     dy = p2[1] - p1[1]
#
#     dp = p2-p1
#
#     if abs(dx) > abs(dy):
#         steps = abs(dx)
#     else:
#         steps = abs(dy)
#
#     xIncrement = dx / float(steps)
#     yIncrement = dy / float(steps)
#
#     x = p1[0]
#     y = p1[1]
#     for i in range(int(steps)):
#         x = x + xIncrement
#         y = y + yIncrement
#         pixels.append((int(x), int(y)))
#     return pixels

class CollisionChecker:

    def __init__(self, img):
        self.img = img

    def get_coor_before_collision(self, posA, posB):
        pixels = get_line(posA, posB)
        # check that all pixel are white (free space)
        endPos = posB
        for p in pixels:
            endPos = (p[0], p[1])
            color = self.img.get_at(endPos)
            if color != (255, 255, 255) and color != (255, 255, 255, 255):
                break
        return endPos

    def path_is_free(self, posA, posB):
        white = 255, 255, 255
        # get list of pixel between node A and B
        # pixels = lineGenerationAlgorithm(posA, posB)
        pixels = get_line(posA, posB)

        # check that all pixel are white (free space)
        for p in pixels:
            color = self.img.get_at((p[0], p[1]))
            if color != (255, 255, 255) and color != (255, 255, 255, 255):
                return False
        return True
