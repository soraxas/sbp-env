import numpy as np
import random
import pygame
import scipy as sp
import scipy.ndimage
import math

from randomPolicySampler import RandomPolicySampler
from checkCollision import get_line


class Particle:
    def __init__(self, direction, pos):
        # self.energy = 1
        self.direction = direction
        self.pos = np.copy(pos)

        self._trying_this_pos = np.copy(pos)
        self._trying_this_dir = None

        self.nothing_to_confirm = False  # NOTE false because at the begining need to confirm starting point

    def try_new_pos(self, new_pos, new_dir):
        self.nothing_to_confirm = False

        self.last_time_already_reverted = False
        # pos is a 2 index list-type object
        self._trying_this_pos[0] = new_pos[0]
        self._trying_this_pos[1] = new_pos[1]
#################################################################################
        self.last_time_already_reverted = False
        # new_dir is a scalar (for now TODO make it to general dimension later)
        self._trying_this_dir = self.direction
        self.direction = new_dir

        return self._trying_this_pos

    def confirm(self):
        if self.nothing_to_confirm:
            raise Exception("This particle is trying to confirm nothingness")
        self.nothing_to_confirm = True
        # to confirm the previous step is valid
        self.pos[0] = self._trying_this_pos[0]
        self.pos[1] = self._trying_this_pos[1]
        self.direction = self._trying_this_dir


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
        self.startPt = kwargs['startPt']
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(XDIM=self.XDIM, YDIM=self.YDIM, RRT=self.RRT)
        # probability layer
        self.particles_layer = pygame.Surface((self.XDIM*self.scaling, self.YDIM*self.scaling),
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
        self.particles_dir = []
        self.particles_weights = np.ones(self.MAX)

        self.gauss_draws = None
        self.gauss_draws_idx = 0

        self.particles = []

        self.NUM_PARTICLES = 10
        for i in range(self.NUM_PARTICLES):
            self.particles.append(Particle(direction=random.uniform(0, math.pi * 2),
                                           pos=self.startPt))


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
            self.particles_weights[idx] *= 0.8

    def reportSuccess(self, idx):
        self.particles[idx].confirm()
        self.particles_weights[idx] *= 1.1

    def randomWalk(self, idx):
        new_direction = self.drawNormal(self.particles[idx].direction)

        factor = self.EPSILON * 1
        x = self.particles[idx].pos[0] + math.cos(new_direction) * factor
        y = self.particles[idx].pos[1] + math.sin(new_direction) * factor

        trying_this = self.particles[idx].try_new_pos((x, y), new_direction)
        return trying_this


    def getNextNode(self):
        print("get")
        if random.random() < 0:
            # if self.prob_vector_normalized is None or random.random() < 0.01:
            print('rand')
            p = self.randomSampler.getNextNode()
            choice = -1
        else:
            # get a node to random walk
            prob = self.particles_weights[0:len(
                self.particles)] / self.particles_weights[0:len(
                    self.particles)].sum()
            # print(prob)
            choice = np.random.choice(range(len(self.particles)), p=prob)

            p = self.randomWalk(choice)

        self.last_particle = p
        # self.particles.append(p)
        return p, choice
        # print(p)
        # p = random.random()*self.XDIM, random.random()*self.YDIM
        # if not self.RRT.collides(p):

    def addTreeNode(self, x, y):
        return

    def addSampleLine(self, x1, y1, x2, y2):
        return

    def addSample(self, **kwargs):
        return


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

        for p in self.particles:
            self.particles_layer.fill((255, 128, 255, 0))
            color = (20,200,200)
            pygame.draw.circle(self.particles_layer, color, p.pos*self.scaling, 4*self.scaling)
            window.blit(self.particles_layer, (0, 0))
