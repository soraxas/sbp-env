import random

from overrides import overrides

from planners.baseSampler import Sampler
from randomness import SUPPORTED_RANDOM_METHODS, RandomnessManager


class RandomPolicySampler(Sampler):

    @overrides
    def __init__(self, random_method='pseudo_random'):
        if random_method not in SUPPORTED_RANDOM_METHODS:
            print("Given random_method is not valid! Valid options includes:\n"
                  "{}".format('\n'.join((' - {}'.format(m) for m in SUPPORTED_RANDOM_METHODS))))
            import sys
            sys.exit(1)
        self.random_method = random_method
        self.random = None

    @overrides
    def init(self, *args, **kwargs):
        self.random = RandomnessManager(num_dim=kwargs['num_dim'])

    @overrides
    def get_next_pos(self):
        # Random path
        if random.random() < self.args.goalBias:
            # goal bias
            p = self.goal_pos
        else:
            p = self.random.get_random(self.random_method)
            p *= self.args.env.dim
        return p, self.report_success, self.report_fail
