import pygame
from pygame.locals import MOUSEBUTTONDOWN
from baseSampler import Sampler
"""
For demo / testing only. This policy wait for user mouse input for next sampling node.
"""

class MouseSampler(Sampler):

    def init(self, **kwargs):
        self.XDIM = kwargs['XDIM']
        self.YDIM = kwargs['YDIM']
        self.RRT = kwargs['RRT']
        self.SCALING = kwargs['SCALING']

    def getNextNode(self):
        next_node = None
        while next_node is None:
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mousePos = (int(e.pos[0] / self.SCALING), int(e.pos[1] / self.SCALING))
                    if next_node is None:
                        next_node = mousePos
        return next_node, self.reportSuccess, self.reportFail
