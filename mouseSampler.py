import pygame
import time
from baseSampler import Sampler
"""
For demo / testing only. This policy wait for user mouse input for next sampling node.
"""

class MouseSampler(Sampler):

    def init(self, **kwargs):
        self.SCALING = kwargs['SCALING']

    def getMouseClickPosition(self):
        while True:
            time.sleep(0.05)
            pygame.event.wait()
            if pygame.mouse.get_pressed()[0] == 1: # Left mouse pressed
                pos = pygame.mouse.get_pos()
                pos = (int(pos[0] / self.SCALING), int(pos[1] / self.SCALING))
                return pos

    def getNextNode(self):
        return self.getMouseClickPosition(), self.reportSuccess, self.reportFail
