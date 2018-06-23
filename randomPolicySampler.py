import numpy as np
import random
from overrides import overrides
from baseSampler import Sampler

class RandomPolicySampler(Sampler):

    @overrides
    def get_next_node(self):
        # Random path
        while True:
            p = random.random()*self.XDIM,  random.random()*self.YDIM
            # p = random.random()*self.XDIM, random.random()*self.YDIM
            # if not self.RRT.collides(p):
            return p, self.report_success, self.report_fail

#############################
# FOR Informed RRT
#############################
#     if self.c_max != INFINITE: #max size represent infinite (not found solution yet)
#         while True:
#             # already have a valid solution, optimise in ellipse region
#             r1 = self.c_max / 2
#             r2 = math.sqrt(abs(self.c_max**2 - self.c_min**2))
#
#             x = np.random.uniform(-1, 1)
#             y = np.random.uniform(-1, 1)
#
#             x2 =  x * r1 * math.cos(self.angle) + y * r2 * math.sin(self.angle)
#             y2 = -x * r1 * math.sin(self.angle) + y * r2 * math.cos(self.angle)
#
#             ##################################
#             ##################################
#             ##################################
#             pos =  x2 + self.x_center[0] , y2 + self.x_center[1]
#             if not self.collides(pos):
#                 return pos
