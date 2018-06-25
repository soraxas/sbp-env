import numpy as np
import random
from overrides import overrides
from baseSampler import Sampler

class RandomPolicySampler(Sampler):

    @overrides
    def get_next_node(self):
        # Random path
        while True:
            if random.random() < self.goalBias:
                # goal bias
                p = self.goalPt
            else:
                p = random.random()*self.XDIM,  random.random()*self.YDIM
            return p, self.report_success, self.report_fail
