import numpy as np
import random

class Sampler:
    """
    Base sampler that defines each unique methods that some
    sampler uses but not all. This sampler does nothing with its own.
    """

    def init(self, *argv, **kwargs):
        pass

    def getNextNode(self, *argv, **kwargs):
        pass

    def addSample(self, *argv, **kwargs):
        pass

    def reportSuccess(self, *argv, **kwargs):
        pass

    def reportFail(self, *argv, **kwargs):
        pass

    def addTreeNode(self, *argv, **kwargs):
        pass

    def addSampleLine(self, *argv, **kwargs):
        pass

    def paint(self, *argv, **kwargs):
        pass
