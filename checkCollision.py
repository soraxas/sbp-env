
def lineGenerationAlgorithm(p1, p2):
    pixels = []
    # edge case of p1 == p2
    if p1 == p2:
        return []
    # given 2 points, return a list that contain all pixel within the two point
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

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


def checkIntersect(nodeA,nodeB,img):
    white = 255, 255, 255
    # get list of pixel between node A and B
    pixels = lineGenerationAlgorithm(nodeA.pos, nodeB.pos)

    # check that all pixel are white (free space)
    for p in pixels:
        color = img.get_at((p[0], p[1]))
        if color != (255, 255, 255) and color != (255, 255, 255, 255):
            return False
    return True
