import numpy as np
import random

class Sampler:
    """
    Base sampler that defines each unique methods that some
    sampler uses but not all. This sampler does nothing with its own.
    """

    def init(self, *argv, **kwargs):
        pass

    def get_next_node(self, *argv, **kwargs):
        pass

    def add_sample(self, *argv, **kwargs):
        pass

    def report_success(self, *argv, **kwargs):
        pass

    def report_fail(self, *argv, **kwargs):
        pass

    def add_tree_node(self, *argv, **kwargs):
        pass

    def add_sample_line(self, *argv, **kwargs):
        pass

    def paint(self, *argv, **kwargs):
        pass
