import math

import numpy as np
import scipy
from SALib.sample import (
    saltelli,
    sobol_sequence,
    latin,
    finite_diff,
    fast_sampler,
)

NUM_DATA_POINTS = 10000
SUPPORTED_RANDOM_METHODS_TITLES = {
    'pseudo_random' : "Pseudo Random",
    'sobol_sequence' : "Sobol sequence",
    'saltelli' : "Saltelli's extension of Sobol sequence",
    'latin_hypercube' : "Latin hypercube",
    'finite_differences' : "Finite differences",
    'fast' : "Fourier Amplitude Sensitivity Test (FAST)",
}
SUPPORTED_RANDOM_METHODS = tuple(t for t in SUPPORTED_RANDOM_METHODS_TITLES)

class NormalRandomnessManager:
    def __init__(self):
        # draws of normal distribution
        self.normal_draws_reserve = None
        # draws of half normal distribution
        self.half_normal_draws_reserve = None

    def redraw_normal(self, kappa, sigma, use_vonmises=True):
        if use_vonmises:
            dist = np.random.vonmises(0, kappa, NUM_DATA_POINTS)
        else:
            dist = np.random.normal(0, sigma, NUM_DATA_POINTS)
        self.normal_draws_reserve = dist

    def draw_normal(self, origin, use_vonmises=True, kappa=1, sigma=math.pi / 4):
        if (self.normal_draws_reserve is None or
            self.normal_draws_reserve.size < 1):
            self.redraw_normal(use_vonmises=True, kappa=1, sigma=math.pi / 4)
        # draw from samples
        draw = self.normal_draws_reserve[-1]
        self.normal_draws_reserve = self.normal_draws_reserve[:-1]
        # shift location
        return draw + origin

    def redraw_half_normal(self, start_at, scale):
        dist = scipy.stats.halfnorm.rvs(loc=start_at, scale=scale, size=NUM_DATA_POINTS)
        self.half_normal_draws_reserve = dist

    def draw_half_normal(self, start_at, scale=1):
        if (self.half_normal_draws_reserve is None or
            self.half_normal_draws_reserve.size < 1):
            self.redraw_half_normal(start_at, scale)
        # draw from samples
        draw = self.half_normal_draws_reserve[-1]
        self.half_normal_draws_reserve = self.half_normal_draws_reserve[:-1]
        return draw


class RandomnessManager:
    def __init__(self):
        # draws of random numbers
        self.random_draws = {}

    def redraw(self, random_method):
        problem = {
            'num_vars': 2,
            'names': ['x', 'y'],
            'bounds': [[0, 1]] * 2
        }
        if random_method == 'pseudo_random':
            seq = np.random.random((NUM_DATA_POINTS, 2))
        elif random_method == 'sobol_sequence':
            seq = sobol_sequence.sample(NUM_DATA_POINTS, 2)
        elif random_method == 'saltelli':
            seq = saltelli.sample(problem, NUM_DATA_POINTS, calc_second_order=False)
        elif random_method == 'latin_hypercube':
            seq = latin.sample(problem, NUM_DATA_POINTS)
        elif random_method == 'finite_differences':
            seq = finite_diff.sample(problem, NUM_DATA_POINTS)
        elif random_method == 'fast':
            seq = fast_sampler.sample(problem, NUM_DATA_POINTS, M=45)
        self.random_draws[random_method] = seq

    def get_random(self, random_method):
        if (random_method not in self.random_draws or
            self.random_draws[random_method].size < 1):
            self.redraw(random_method)
        last = self.random_draws[random_method][-1]
        self.random_draws[random_method] = self.random_draws[random_method][:-1]
        return last

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
            title = SUPPORTED_RANDOM_METHODS_TITLES[m]
            random_numbers.redraw(m)
            seq = random_numbers.random_draws[m][:NUM_DATA_POINTS]
            show_fig(seq.T[0], seq.T[1], title)
