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
    "pseudo_random": "Pseudo Random",
    "sobol_sequence": "Sobol sequence",
    "saltelli": "Saltelli's extension of Sobol sequence",
    "latin_hypercube": "Latin hypercube",
    "finite_differences": "Finite differences",
    "fast": "Fourier Amplitude Sensitivity Test (FAST)",
}
SUPPORTED_RANDOM_METHODS = tuple(t for t in SUPPORTED_RANDOM_METHODS_TITLES)
"""Supported methods to draw random numbers in samplers. Supported random methods are 
as follows:

* ``pseudo_random`` from numpy
    **Pseudo Random**
* ``sobol_sequence`` from *SALib*
    **Sobol sequence**
* ``saltelli`` from *SALib*
    **Saltelli's extension of Sobol sequence**
* ``latin_hypercube`` from *SALib*
    **Latin hypercube**
* ``finite_differences`` from *SALib*
    **Finite differences**
* ``fast`` from *SALib*
    **Fourier Amplitude Sensitivity Test (FAST)**

"""


class NormalRandomnessManager:
    """
    Randomness Manager that draw a bucket of **normally distributed** random numbers
    and refill the bucket on-demand when it is exhausted.
    """

    def __init__(self):
        # draws of normal distribution
        self.normal_draws_reserve = None
        # draws of half normal distribution
        self.half_normal_draws_reserve = None

    def redraw_normal(self, kappa, sigma, use_vonmises=True):
        r"""Redraw the bucket of normally distributed random numbers

        :param kappa: the kappa :math:`\kappa` parameter to be used to draw from a
            von Mises distribution

            .. math::
                x \sim f_\text{VonMises}( x | \mu, \kappa)

        :param sigma: the sigma :math:`\sigma` parameter to be used to draw from a
            normal distribution
            .. math::
                x \sim \mathcal{N}( x | \mu, \sigma^2)

        :param use_vonmises:  (Default value = True)

        """
        if use_vonmises:
            assert kappa is not None
            dist = np.random.vonmises(0, kappa, NUM_DATA_POINTS)
        else:
            assert sigma is not None
            dist = np.random.normal(0, sigma, NUM_DATA_POINTS)
        self.normal_draws_reserve = dist

    # def draw_normal(
    #     self,
    #     origin: np.ndarray,
    #     use_vonmises: bool = True,
    #     kappa: float = 1,
    #     sigma: float = math.pi / 4,
    # ) -> np.ndarray:
    #     """
    #
    #     :param origin: the origin (mean) for the random number, which will be used to
    #     shift the number that this function returns
    #     :param use_vonmises:  (Default value = True)
    #     :param kappa:  (Default value = 1)
    #     :param sigma:  (Default value = math.pi / 4)
    #
    #     """
    #     if self.normal_draws_reserve is None or self.normal_draws_reserve.size < 1:
    #         self.redraw_normal(
    #             use_vonmises=use_vonmises, kappa=kappa, sigma=math.pi / 4
    #         )
    #     # draw from samples
    #     draw = self.normal_draws_reserve[-1]
    #     self.normal_draws_reserve = self.normal_draws_reserve[:-1]
    #     # shift location
    #     return draw + origin

    def redraw_half_normal(self, start_at, scale):
        """

        :param start_at: 
        :param scale: 

        """
        dist = scipy.stats.halfnorm.rvs(loc=start_at, scale=scale, size=NUM_DATA_POINTS)
        self.half_normal_draws_reserve = dist

    def draw_half_normal(self, start_at, scale=1):
        """

        :param start_at: 
        :param scale:  (Default value = 1)

        """
        if (
            self.half_normal_draws_reserve is None
            or self.half_normal_draws_reserve.size < 1
        ):
            self.redraw_half_normal(start_at, scale)
        # draw from samples
        draw = self.half_normal_draws_reserve[-1]
        self.half_normal_draws_reserve = self.half_normal_draws_reserve[:-1]
        return draw


class RandomnessManager:
    """ """

    def __init__(self, num_dim):
        # draws of random numbers
        self.random_draws = {}
        self.num_dim = num_dim

    def redraw(self, random_method):
        """

        :param random_method: 

        """
        problem = {
            "num_vars": self.num_dim,
            "names": list(range(self.num_dim)),
            "bounds": [[0, 1]] * self.num_dim,
        }
        if random_method == "pseudo_random":
            seq = np.random.random((NUM_DATA_POINTS, 2))
        elif random_method == "sobol_sequence":
            seq = sobol_sequence.sample(NUM_DATA_POINTS, 2)
        elif random_method == "saltelli":
            seq = saltelli.sample(problem, NUM_DATA_POINTS, calc_second_order=False)
        elif random_method == "latin_hypercube":
            seq = latin.sample(problem, NUM_DATA_POINTS)
        elif random_method == "finite_differences":
            seq = finite_diff.sample(problem, NUM_DATA_POINTS)
        elif random_method == "fast":
            seq = fast_sampler.sample(problem, NUM_DATA_POINTS, M=45)
        self.random_draws[random_method] = seq

    def get_random(self, random_method):
        """Get one sample of random number :math:`r` where :math:`0 \lt r \lt 1`

        :param random_method: The kind of random number method to use, must be one of
        the choice in :data:`SUPPORTED_RANDOM_METHODS`

        """
        if (
            random_method not in self.random_draws
            or self.random_draws[random_method].size < 1
        ):
            self.redraw(random_method)
        last = self.random_draws[random_method][-1]
        self.random_draws[random_method] = self.random_draws[random_method][:-1]
        return last


if __name__ == "__main__":
    # show presentation of plotting different qrsai-random numbers
    import matplotlib.pyplot as plt
    from matplotlib.pyplot import figure

    def show_fig(x, y, title=None):
        """

        :param x: 
        :param y: 
        :param title:  (Default value = None)

        """
        figure(num=1, figsize=(8, 6), dpi=200)
        plt.title(title)
        plt.plot(x, y, "r.")
        plt.show()

    random_numbers = RandomnessManager(num_dim=2)
    for _ in range(1):
        for m in SUPPORTED_RANDOM_METHODS:
            title = SUPPORTED_RANDOM_METHODS_TITLES[m]
            random_numbers.redraw(m)
            seq = random_numbers.random_draws[m][:NUM_DATA_POINTS]
            show_fig(seq.T[0], seq.T[1], title)
