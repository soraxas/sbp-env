
import numpy as np
import random
import pygame
import scipy as sp
import scipy.ndimage

from randomPolicySampler import RandomPolicySampler
from checkCollision import get_line

class LikelihoodPolicySampler:

    def __init__(self, prob_block_size, supressVisitedArea=True):
        self.PROB_BLOCK_SIZE = prob_block_size
        self.supressVisitedArea = supressVisitedArea

    def init(self, **kwargs):
        self.XDIM = kwargs['XDIM']
        self.YDIM = kwargs['YDIM']
        self.RRT = kwargs['RRT']
        self.scaling = kwargs['SCALING']
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(XDIM=self.XDIM, YDIM=self.YDIM, RRT=self.RRT)
        # probability layer
        self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * self.scaling,self.PROB_BLOCK_SIZE * self.scaling), pygame.SRCALPHA)

        shape = (int(self.XDIM/self.PROB_BLOCK_SIZE) + 1, int(self.YDIM/self.PROB_BLOCK_SIZE) + 1 )
        self.prob_vector = np.ones(shape)
        self.prob_vector *= 20
        self.prob_vector_normalized = None
        self.tree_vector = np.ones(shape)

        self.sampleCount = 0

    def getNextNode(self):
        if self.prob_vector_normalized is None:
            return self.randomSampler.getNextNode()
        while True:
            choice = np.random.choice(range(self.prob_vector_normalized.size), p=self.prob_vector_normalized.ravel())
            y = choice % self.prob_vector_normalized.shape[1]
            x = int(choice / self.prob_vector_normalized.shape[1])

            p = (x + random.random())*self.PROB_BLOCK_SIZE, (y + random.random())*self.PROB_BLOCK_SIZE
            # print(p)
            # p = random.random()*self.XDIM, random.random()*self.YDIM
            if not self.RRT.collides(p):
                return np.array(p)

    def addTreeNode(self, x, y):
        x = int(x / self.PROB_BLOCK_SIZE)
        y = int(y / self.PROB_BLOCK_SIZE)
        self.tree_vector[x][y] += 1

    def addSampleLine(self, x1, y1, x2, y2):
        x1 = int(x1 / self.PROB_BLOCK_SIZE)
        y1 = int(y1 / self.PROB_BLOCK_SIZE)
        x2 = int(x2 / self.PROB_BLOCK_SIZE)
        y2 = int(y2 / self.PROB_BLOCK_SIZE)
        points = get_line((x1,y1), (x2,y2))
        for p in points:
            self.addSample(p=p, free=True, alreadyDividedByProbBlockSize=True)

    def addSample(self, **kwargs):
    # def addInvalidPoint(self,p, blockedSpace, perma=False, alreadyDividedByProbBlockSize=False):
        p = kwargs['p']
        if p is None:
            return
        try:
            p = p.pos
        except AttributeError as e:
            pass

        if True:
            if 'alreadyDividedByProbBlockSize' not in kwargs:
            # if not alreadyDividedByProbBlockSize:
                x = int(p[0]/self.PROB_BLOCK_SIZE)
                y = int(p[1]/self.PROB_BLOCK_SIZE)
            else:
                x = p[0]
                y = p[1]
            if x < 0 or x >= self.prob_vector.shape[0] or \
                y < 0 or y >= self.prob_vector.shape[1]:
                    return

            factor = 1.5

            if 'obstacle' in kwargs or not kwargs['free']:
                self.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
                if self.prob_vector[x][y] < 5:
                    self.prob_vector[x][y] = 5
            else:
                self.prob_vector[x][y] = 100
                if False:
                        self.prob_vector[x][y] += (100-self.prob_vector[x][y])*0.5
                        if self.prob_vector[x][y] > 100:
                            self.prob_vector[x][y] = 100

    #########################################################

            sigma_y = 1.0
            sigma_x = 1.0
            sigma = [sigma_y, sigma_x]
            if self.sampleCount % 200 == 0:
                pass
                self.prob_vector_normalized = np.copy(self.prob_vector)
                self.prob_vector_normalized *= 1/self.tree_vector
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(self.prob_vector_normalized, sigma, mode='reflect')
                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            self.sampleCount += 1


#########################################################
#### FOR PAINTING
#########################################################

    def get_vector_alpha_parameters(self, vector):
        max_prob = vector.max()
        min_prob = vector.min()
        denominator = max_prob-min_prob
        if denominator == 0:
            denominator = 1 # prevent division by zero
        return max_prob, min_prob, denominator


    def paint(self, window):
        if self.prob_vector_normalized is not None:
            for i in range(self.prob_vector_normalized.shape[0]):
                for j in range(self.prob_vector_normalized.shape[1]):
                    max_prob, min_prob, denominator = self.get_vector_alpha_parameters(self.prob_vector_normalized)
                    alpha = 240 * (1 -(self.prob_vector_normalized[i][j]-min_prob)/denominator)

                    # if self.tree_vector[i][j] > 1:
                    #     max_prob, min_prob, denominator = self.get_vector_alpha_parameters(self.tree_vector)
                    #     alpha = 240 * (1 - (self.prob_vector_normalized[i][j]-min_prob)/denominator)
                    #     print(alpha)
                    #     self.prob_layer.fill((0,255,0,alpha))
                    # else:
                    self.prob_layer.fill((255,128,255,alpha))
                    # print(self.prob_vector_normalized[i][j])
                    window.blit(self.prob_layer, (i*self.PROB_BLOCK_SIZE*self.scaling,j*self.PROB_BLOCK_SIZE*self.scaling))
