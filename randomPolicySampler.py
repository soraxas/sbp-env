import numpy as np
import random
from overrides import overrides
from baseSampler import Sampler
from SALib.sample import (
    saltelli,
    sobol_sequence,
    latin,
    finite_diff,
    fast_sampler,
    ff,
)

NUM_DATA_POINTS = 10000
SUPPORTED_RANDOM_METHODS = (
    'pseudo_random',
    'sobol_sequence',
    'saltelli',
    'latin_hypercube',
    'finite_differences',
    'fast'
    )
PROBLEM = {
    'num_vars': 2,
    'names': ['x', 'y'],
    'bounds': [[0, 1]] * 2
}
class RandomnessManager:
    def __init__(self):
        # draws of random numbers
        self.random_draws = {}

    def redraw(self, random_method):

        if random_method == 'pseudo_random':
            seq = np.random.random((NUM_DATA_POINTS, 2))
        elif random_method == 'sobol_sequence':
            seq = sobol_sequence.sample(NUM_DATA_POINTS, 2)
        elif random_method == 'saltelli':
            seq = saltelli.sample(PROBLEM, NUM_DATA_POINTS, calc_second_order=False)
        elif random_method == 'latin_hypercube':
            seq = latin.sample(PROBLEM, NUM_DATA_POINTS)
        elif random_method == 'finite_differences':
            seq = finite_diff.sample(PROBLEM, NUM_DATA_POINTS)
        elif random_method == 'fast':
            seq = fast_sampler.sample(PROBLEM, NUM_DATA_POINTS, M=45)
        self.random_draws[random_method] = seq

    def get_random(self, random_method):
        if (random_method not in self.random_draws or
            self.random_draws[random_method].size < 1):
            self.redraw(random_method)
        last = self.random_draws[random_method][-1]
        self.random_draws[random_method] = self.random_draws[random_method][:-1]
        return last


class RandomPolicySampler(Sampler):

    @overrides
    def __init__(self, random_method):
        if random_method not in SUPPORTED_RANDOM_METHODS:
            print("Given random_method is not valid! Valid options includes:\n"
                  "{}".format('\n'.join((' - {}'.format(m) for m in SUPPORTED_RANDOM_METHODS))))
            import sys
            sys.exit(1)
        self.random_method = random_method
        self.random = RandomnessManager()

    @overrides
    def get_next_node(self):
        # Random path
        while True:
            if random.random() < self.goalBias:
                # goal bias
                p = self.goalPt
            else:
                p = self.random.get_random(self.random_method)
                p[0] *= self.XDIM
                p[1] *= self.YDIM
            return p, self.report_success, self.report_fail


if __name__ == '__main__':
    # show presentation of plotting different qrsai-random numbers
    import matplotlib.pyplot as plt
    from matplotlib.pyplot import figure

    def show_fig(x, y, title=None):
        figure(num=1, figsize=(8, 6), dpi=200)
        plt.title(title)
        plt.plot(x, y, 'r.')
        plt.show()

    random_numbers = RandomnessManager()
    for _ in range(1):
        for m in SUPPORTED_RANDOM_METHODS:
            title = {
                'pseudo_random' : "Pseudo Random",
                'sobol_sequence' : "Sobol sequence",
                'saltelli' : "Saltelli's extension of Sobol sequence",
                'latin_hypercube' : "Latin hypercube",
                'finite_differences' : "Finite differences",
                'fast' : "Fourier Amplitude Sensitivity Test (FAST)",
            }[m]
            random_numbers.redraw(m)
            seq = random_numbers.random_draws[m][:NUM_DATA_POINTS]
            show_fig(seq.T[0], seq.T[1], title)
