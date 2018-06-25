import pygame
import time
from overrides import overrides
from baseSampler import Sampler
"""
For demo / testing only. This policy wait for user mouse input for next sampling node.
"""

class MouseSampler(Sampler):

    @overrides
    def get_next_node(self):
        return self.get_mouse_click_position(scaling=self.scaling), self.report_success, self.report_fail

    @staticmethod
    def get_mouse_click_position(scaling):
        while True:
            time.sleep(0.05)
            pygame.event.wait()
            if pygame.mouse.get_pressed()[0] == 1: # Left mouse pressed
                pos = pygame.mouse.get_pos()
                pos = (int(pos[0] / scaling), int(pos[1] / scaling))
                return pos