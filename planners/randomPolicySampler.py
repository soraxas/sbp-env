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
        super().init(*args, **kwargs)
        self.random = RandomnessManager(num_dim=kwargs['num_dim'])

    @overrides
    def get_next_pos(self):
        # Random path
        if random.random() < self.args.goalBias:
            # goal bias
            p = self.goal_pos
        else:
            if self.use_radian:
                import numpy as np

                low, high = ([-3.12413936106985, -2.5743606466916362, -2.530727415391778,
                              -3.12413936106985, -2.443460952792061, -3.12413936106985],
                             [3.12413936106985, 2.2689280275926285, 2.530727415391778,
                              3.12413936106985, 2.007128639793479, 3.12413936106985])
                p = np.random.uniform(low, high)

            else:
                p = self.random.get_random(self.random_method)
                p *= self.args.env.dim

        return p, self.report_success, self.report_fail
