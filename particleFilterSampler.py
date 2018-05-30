import numpy as np
import random
import pygame
import scipy as sp
import scipy.ndimage
import math

from randomPolicySampler import RandomPolicySampler
from checkCollision import get_line


class Particle:
    def __init__(self):
        self.energy = 1
        self.direction = None

class ParticleFilterSampler:
    def __init__(self, prob_block_size, supressVisitedArea=True):
        self.PROB_BLOCK_SIZE = prob_block_size
        self.supressVisitedArea = supressVisitedArea


    def init(self, **kwargs):
        self.XDIM = kwargs['XDIM']
        self.YDIM = kwargs['YDIM']
        self.RRT = kwargs['RRT']
        self.EPSILON = kwargs['EPSILON']
        self.scaling = kwargs['SCALING']
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(XDIM=self.XDIM, YDIM=self.YDIM, RRT=self.RRT)
        # probability layer
        self.prob_layer = pygame.Surface((self.PROB_BLOCK_SIZE * self.scaling,
                                          self.PROB_BLOCK_SIZE * self.scaling),
                                         pygame.SRCALPHA)

        shape = (int(self.XDIM / self.PROB_BLOCK_SIZE) + 1,
                 int(self.YDIM / self.PROB_BLOCK_SIZE) + 1)
        self.prob_vector = np.ones(shape)
        self.prob_vector *= 1  # IMPORTANT because we are using log2
        self.obst_vector = np.ones(shape)
        # self.prob_vector *= 20
        self.prob_vector_normalized = None
        self.tree_vector = np.ones(shape)

        self.sampleCount = 0

        self.MAX = 10000
        self.particles = []
        self.particles_dir = []
        self.particles_weights = np.ones(self.MAX)

        self.gauss_draws = None
        self.gauss_draws_idx = 0


    def drawNormal(self, origin):
        if self.gauss_draws is None or self.gauss_draws_idx + 2 >= self.gauss_draws.size:
            # redraw
            mu, sigma = 0, math.pi / 4
            self.gauss_draws_idx = 0
            self.gauss_draws = np.random.normal(mu, sigma, 1000)
        # draw from samples
        loc = self.gauss_draws[self.gauss_draws_idx]
        self.gauss_draws_idx += 1
        # shift location
        loc += origin
        return loc

    def reportFail(self, idx):
        if idx >= 0:
            self.particles_weights[idx] *= 0.5

    def reportSuccess(self, node):
        self.particles.append(self.last_particle)
        self.particles_dir.append(self._last_dir)

    def setStartPt(self, pos):
        self.particles.append(pos)
        self.particles_dir.append(0)  # TODO: make it random

    def drewedParent(self, idx):
        self.particles_weights[idx] *= 0.85

    def getNextNode(self):
        if len(self.particles) - 5 >= self.MAX:
            print("TOO MANY PARTICLESSSSSSSSSSS")
            import sys
            sys.exit(1)
            return np.array(p)

        if random.random() < 0:
            # if self.prob_vector_normalized is None or random.random() < 0.01:
            print('rand')
            p = self.randomSampler.getNextNode()
            choice = -1
        else:
            prob = self.particles_weights[0:len(
                self.particles)] / self.particles_weights[0:len(
                    self.particles)].sum()
            print(len(self.particles))
            print(prob.size)
            choice = np.random.choice(range(len(self.particles)), p=prob)
            print(self.particles[choice])
            self.drewedParent(choice)

            # get a random direction, spawn a new particle
            parent_direction = self.particles_dir[choice]
            new_direction = self.drawNormal(parent_direction)

            self._last_dir = new_direction

            factor = self.EPSILON * 4
            x = self.particles[choice][0] + math.cos(new_direction) * factor
            y = self.particles[choice][1] + math.sin(new_direction) * factor

            # p = (x + random.random())*self.PROB_BLOCK_SIZE, (y + random.random())*self.PROB_BLOCK_SIZE
            p = np.array([x, y])
            print(p)

        self.last_particle = p
        # self.particles.append(p)
        print(p)
        return p, choice
        # print(p)
        # p = random.random()*self.XDIM, random.random()*self.YDIM
        # if not self.RRT.collides(p):

    def addTreeNode(self, x, y):
        x = int(x / self.PROB_BLOCK_SIZE)
        y = int(y / self.PROB_BLOCK_SIZE)
        self.tree_vector[x][y] += 1

    def addSampleLine(self, x1, y1, x2, y2):
        x1 = int(x1 / self.PROB_BLOCK_SIZE)
        y1 = int(y1 / self.PROB_BLOCK_SIZE)
        x2 = int(x2 / self.PROB_BLOCK_SIZE)
        y2 = int(y2 / self.PROB_BLOCK_SIZE)
        points = get_line((x1, y1), (x2, y2))
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
        if 'alreadyDividedByProbBlockSize' not in kwargs:
            # if not alreadyDividedByProbBlockSize:
            x = int(p[0] / self.PROB_BLOCK_SIZE)
            y = int(p[1] / self.PROB_BLOCK_SIZE)
        else:
            x = p[0]
            y = p[1]
        if x < 0 or x >= self.prob_vector.shape[0] or \
            y < 0 or y >= self.prob_vector.shape[1]:
            return

        # print(p)
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
            # print(p)
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
                tree_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    tree_vector_normalized, (1.0, 1.0), mode='reflect')
                # self.prob_vector_normalized = tree_vector_normalized
                self.prob_vector_normalized *= (1 / self.obst_vector * 3)
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    self.prob_vector_normalized, sigma, mode='reflect')
                self.prob_vector_normalized *= (
                    1 / tree_vector_normalized * 1.5)
                # self.prob_vector_normalized *= (1/self.tree_vector * 1.5)
                self.prob_vector_normalized /= self.prob_vector_normalized.sum(
                )
            self.sampleCount += 1


#########################################################
#### FOR PAINTING
#########################################################

    def get_vector_alpha_parameters(self, vector):
        max_prob = vector.max()
        min_prob = vector.min()
        denominator = max_prob - min_prob
        if denominator == 0:
            denominator = 1  # prevent division by zero
        return max_prob, min_prob, denominator

    def paint(self, window):
        return

        if self.prob_vector_normalized is not None:
            for i in range(self.prob_vector_normalized.shape[0]):
                for j in range(self.prob_vector_normalized.shape[1]):
                    max_prob, min_prob, denominator = self.get_vector_alpha_parameters(
                        self.prob_vector_normalized)
                    alpha = 240 * (
                        1 - (self.prob_vector_normalized[i][j] - min_prob
                             ) / denominator)

                    # if self.tree_vector[i][j] > 1:
                    #     max_prob, min_prob, denominator = self.get_vector_alpha_parameters(self.tree_vector)
                    #     alpha = 240 * (1 - (self.prob_vector_normalized[i][j]-min_prob)/denominator)
                    #     print(alpha)
                    #     self.prob_layer.fill((0,255,0,alpha))
                    # else:
                    self.prob_layer.fill((255, 128, 255, alpha))
                    # print(self.prob_vector_normalized[i][j])
                    window.blit(self.prob_layer,
                                (i * self.PROB_BLOCK_SIZE * self.scaling,
                                 j * self.PROB_BLOCK_SIZE * self.scaling))
