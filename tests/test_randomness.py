from unittest import TestCase

import numpy as np

from randomness import (
    RandomnessManager,
    SUPPORTED_RANDOM_METHODS,
    NormalRandomnessManager,
)


class TestRandomnessManager(TestCase):
    def test_non_existing_method(self):
        with self.assertRaises(ValueError):
            randomness = RandomnessManager(2)
            randomness.get_random("my_non_existing_random_method")

    def test_all_supported_methods(self):
        randomness = RandomnessManager(2, bucket_size=10000)

        # loop through all supported methods
        for method in SUPPORTED_RANDOM_METHODS:
            for i in range(10):
                randomness.get_random(method)

    def test_all_supported_methods_higher_dimensions(self):

        for num_dim in range(2, 10):
            # loop through all supported methods
            for method in SUPPORTED_RANDOM_METHODS:
                # NOTE: the fast sampler random_method requires some minimum sample size
                randomness = RandomnessManager(
                    num_dim, bucket_size=10 if method != "fast" else 10000
                )
                for i in range(10):
                    randomness.get_random(method)

    def test_redraw(self):
        # test drawing more than NUM_DATA_POINTS to see if it will automatically refill

        test_bucket_size = 10
        randomness = RandomnessManager(2, bucket_size=test_bucket_size)

        method = "pseudo_random"
        randomness.get_random(method)

        # test it is currently one less than the requested size
        self.assertEqual(len(randomness.random_draws[method]), test_bucket_size - 1)

        # exhaust the sampled points
        for i in range(test_bucket_size - 1):
            randomness.get_random(method)

        # ensure all points being exhausted
        self.assertEqual(len(randomness.random_draws[method]), 0)

        # ensure it will refill
        randomness.get_random(method)
        self.assertEqual(len(randomness.random_draws[method]), test_bucket_size - 1)


class TestNormalRandomnessManager(TestCase):
    def test_draw_half_normal(self):
        randomness = NormalRandomnessManager()

        randomness.draw_half_normal(np.array([10]))

    def test_draw_normal(self):
        randomness = NormalRandomnessManager()

        randomness.draw_normal(np.array([10]))

    def test_draw_normal_vonmises(self):
        randomness = NormalRandomnessManager()

        randomness.draw_normal(np.array([10]), use_vonmises=True)
