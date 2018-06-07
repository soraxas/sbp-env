import numpy as np
import random

class Sampler:
    """
    Base sampler that defines each unique methods that some
    sampler uses but not all. This sampler does nothing with its own.
    """

    def init(self, **kwargs):
        pass

    def getNextNode(self):
        pass

    def addSample(self, **kwargs):
        pass

    def reportSuccess(self):
        pass

    def reportFail(self):
        pass

    def addTreeNode(self, x, y):
        pass

    def addSampleLine(self, x1, y1, x2, y2):
        pass

    def paint(self, window):
        pass
