import logging
import random

import numpy as np
import pygame
import scipy as sp
import scipy.ndimage
from overrides import overrides

from checkCollision import get_line
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler

LOGGER = logging.getLogger(__name__)

class LikelihoodPolicySampler(Sampler):

    @overrides
    def __init__(self, prob_block_size, supressVisitedArea=True):
        self.PROB_BLOCK_SIZE = prob_block_size
        self.supressVisitedArea = supressVisitedArea

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)
        # probability layer
        self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * self.args.scaling,self.PROB_BLOCK_SIZE * self.args.scaling), pygame.SRCALPHA)

        self.shape = (int(self.args.XDIM/self.PROB_BLOCK_SIZE) + 1, int(self.args.YDIM/self.PROB_BLOCK_SIZE) + 1 )
        self.prob_vector = np.ones(self.shape)
        self.prob_vector *= 1 # IMPORTANT because we are using log2
        self.obst_vector = np.ones(self.shape)
        # self.prob_vector *= 20
        self.prob_vector_normalized = None
        self.tree_vector = np.ones(self.shape)

        self.sampleCount = 0

    @overrides
    def get_next_pos(self):
        if self.prob_vector_normalized is None or random.random() < 0.05:
            LOGGER.debug('rand')
            p =  self.randomSampler.get_next_pos()[0]
        else:
            choice = np.random.choice(range(self.prob_vector_normalized.size), p=self.prob_vector_normalized.ravel())
            y = choice % self.prob_vector_normalized.shape[1]
            x = int(choice / self.prob_vector_normalized.shape[1])
            p = (x + random.random())*self.PROB_BLOCK_SIZE, (y + random.random())*self.PROB_BLOCK_SIZE
        return p, self.report_success, self.report_fail

    @overrides
    def add_tree_node(self, pos):
        x, y = pos
        x = int(x / self.PROB_BLOCK_SIZE)
        y = int(y / self.PROB_BLOCK_SIZE)
        self.tree_vector[x][y] += 1

    @overrides
    def add_sample_line(self, x1, y1, x2, y2):
        x1 = int(x1 / self.PROB_BLOCK_SIZE)
        y1 = int(y1 / self.PROB_BLOCK_SIZE)
        x2 = int(x2 / self.PROB_BLOCK_SIZE)
        y2 = int(y2 / self.PROB_BLOCK_SIZE)
        points = get_line((x1,y1), (x2,y2))
        for p in points:
            self.report_fail(pos=p, free=True, alreadyDividedByProbBlockSize=True)

    @overrides
    def report_success(self, **kwargs):
        x, y = kwargs['pos']
        # add all in between point of nearest node of the random pt as valid
        x1, y1 = self.args.env.cc.get_coor_before_collision(kwargs['nn'].pos, kwargs['rand_pos'])
        self.add_sample_line(x, y, x1, y1)

    @overrides
    def report_fail(self, **kwargs):
        p = kwargs['pos']
        if p is None:
            return
        try:
            p = p.pos
        except AttributeError as e:
            pass
        if 'alreadyDividedByProbBlockSize' not in kwargs:
            x = int(p[0]/self.PROB_BLOCK_SIZE)
            y = int(p[1]/self.PROB_BLOCK_SIZE)
        else:
            x = p[0]
            y = p[1]
        if x < 0 or x >= self.prob_vector.shape[0] or \
            y < 0 or y >= self.prob_vector.shape[1]:
                return

        # exit()
        # ^ setting the right coor ^
        ###############################
        factor = 1.5

        if 'obstacle' in kwargs:
            self.obst_vector[x][y] += 2
        elif not kwargs['free']:
            self.obst_vector[x][y] += 1
            # self.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
            # if self.prob_vector[x][y] < 5:
                # self.prob_vector[x][y] = 5
        elif kwargs['free']:
            if 'weight' in kwargs:
                self.prob_vector[x][y] += kwargs['weight']
            else:
                self.prob_vector[x][y] += 1
                # self.prob_vector[x][y] = 10
            self.obst_vector[x][y] = 1

    #########################################################

            sigma_y = 2.0
            sigma_x = 2.0
            sigma = [sigma_y, sigma_x]
            if self.sampleCount % 10 == 0:
                pass
                self.prob_vector_normalized = np.copy(self.prob_vector)
                # self.prob_vector_normalized = np.f.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
            # if self.prob_vector[x][y] < 5:
                # self.prob_vector[copy(self.prob_vector)
                tree_vector_normalized = np.copy(self.tree_vector**1.1)
                tree_vector_normalized = sp.ndimage.filters.gaussian_filter(tree_vector_normalized, (1.0,1.0), mode='reflect')
                # self.prob_vector_normalized = tree_vector_normalized
                self.prob_vector_normalized *= (1/self.obst_vector * 3)
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(self.prob_vector_normalized, sigma, mode='reflect')
                self.prob_vector_normalized *= (1/tree_vector_normalized * 1.5)
                # self.prob_vector_normalized *= (1/self.tree_vector * 1.5)
                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            self.sampleCount += 1



#########################################################
#### FOR PAINTING
#########################################################

    @staticmethod
    def get_vector_alpha_parameters(vector):
        max_prob = vector.max()
        min_prob = vector.min()
        denominator = max_prob-min_prob
        if denominator == 0:
            denominator = 1 # prevent division by zero
        return max_prob, min_prob, denominator


    @overrides
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
                    window.blit(self.prob_layer, (i*self.PROB_BLOCK_SIZE*self.args.scaling,j*self.PROB_BLOCK_SIZE*self.args.scaling))
