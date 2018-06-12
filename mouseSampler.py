import pygame
import time
from baseSampler import Sampler
"""
For demo / testing only. This policy wait for user mouse input for next sampling node.
"""

class MouseSampler(Sampler):

    def init(self, **kwargs):
        self.SCALING = kwargs['SCALING']

    def get_mouse_click_position(self):
        while True:
            time.sleep(0.05)
            pygame.event.wait()
            if pygame.mouse.get_pressed()[0] == 1: # Left mouse pressed
                pos = pygame.mouse.get_pos()
                pos = (int(pos[0] / self.SCALING), int(pos[1] / self.SCALING))
                return pos

    def get_next_node(self):
        return self.get_mouse_click_position(), self.report_success, self.report_fail
