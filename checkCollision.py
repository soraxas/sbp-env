import numpy as np


def dist(p1, p2):
    return np.linalg.norm(p1 - p2)
    # return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

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
    x1, y1 = start
    x2, y2 = end
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

def lineGenerationAlgorithm(p1, p2):
    pixels = []
    # edge case of p1 == p2
    if np.array_equal (p1,p2):
        return []
    # given 2 points, return a list that contain all pixel within the two point
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    dp = p2-p1

    if abs(dx) > abs(dy):
        steps = abs(dx)
    else:
        steps = abs(dy)

    xIncrement = dx / float(steps)
    yIncrement = dy / float(steps)

    x = p1[0]
    y = p1[1]
    for i in range(int(steps)):
        x = x + xIncrement
        y = y + yIncrement
        pixels.append((int(x), int(y)))
    return pixels

# import kde
# counter = 0
# kernel = None
# def checkIntersect(nodeA,nodeB,img):
#     # initialise
#     if kernel is None:
#         kernel = kde()
#     # log the answer
#     ans = checkIntersect1(nodeA, nodeB, img)
#     # see if we want to display


def checkIntersect(nodeA,nodeB,img):
    white = 255, 255, 255
    # get list of pixel between node A and B
    # pixels = lineGenerationAlgorithm(nodeA.pos, nodeB.pos)
    pixels = get_line(nodeA.pos.astype(int), nodeB.pos.astype(int))

    # check that all pixel are white (free space)
    for p in pixels:
        color = img.get_at((p[0], p[1]))
        if color != (255, 255, 255) and color != (255, 255, 255, 255):
            return False
    return True
