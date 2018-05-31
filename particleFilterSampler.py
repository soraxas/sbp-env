import numpy as np
import random
import pygame
import scipy as sp
import scipy.ndimage
import math

from randomPolicySampler import RandomPolicySampler
from checkCollision import get_line
"""
    IDEAS / TODOS:

    - Physics engine to bounce off wall
    - Bias toward goalPt
    - Wall following?
    - RESTART
        - Random tree node restart
        - Re-sampling according to particle weight/energy
        - Restart according to tree node
        - Random free space restart (NEED TO CONNECT DIFFERENT TREES TOGETHER)
    - Keep structure of undelying map to restart
"""


def drawNormal(origin):
    if ('normal_dist_draws_reserve' not in drawNormal.__dict__
            or drawNormal.cur_idx + 2 >=
            drawNormal.normal_dist_draws_reserve.size):
        # redraw
        mu, sigma = 0, math.pi / 4
        drawNormal.cur_idx = 0
        drawNormal.normal_dist_draws_reserve = np.random.normal(
            mu, sigma, 1000)
    # draw from samples
    draws = drawNormal.normal_dist_draws_reserve[drawNormal.cur_idx]
    drawNormal.cur_idx += 1
    # shift location
    draws += origin
    return draws


class ParticleManager:
    def __init__(self, num_particles, startPt):
        self.num_particles = num_particles
        self.particles_energy = np.ones(num_particles)
        self.particles = []
        self.cur_energy_sum = self.particles_energy.sum()

        for i in range(self.num_particles):
            self.particles.append(
                Particle(
                    direction=random.uniform(0, math.pi * 2), pos=startPt))

    def size(self):
        return self.num_particles

    def modify_energy(self, idx, factor):
        # keep track how much energy this operation would modify,
        # so we can change the energy_sum accordingly
        old_energy = self.particles_energy[idx]
        self.particles_energy[idx] *= factor

        delta = self.particles_energy[idx] - old_energy
        self.cur_energy_sum += delta

    def confirm(self, idx):
        self.particles[idx].confirm()

    def new_pos(self, idx, pos, dir):
        return self.particles[idx].try_new_pos((pos[0], pos[1]), dir)

    def get_pos(self, idx):
        return self.particles[idx].pos[0], self.particles[idx].pos[1]

    def get_dir(self, idx):
        return self.particles[idx].direction

    def get_prob(self):
        return self.particles_energy / self.cur_energy_sum


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
        self._last_prob = None

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
        self.particles_layer = pygame.Surface(
            (self.XDIM * self.scaling, self.YDIM * self.scaling),
            pygame.SRCALPHA)

        self.p_manager = ParticleManager(
            num_particles=10, startPt=self.startPt)

    def reportFail(self, idx):
        if idx >= 0:
            self.p_manager.modify_energy(idx=idx, factor=0.8)

    def reportSuccess(self, idx):
        self.p_manager.confirm(idx)
        self.p_manager.modify_energy(idx=idx, factor=1.1)

    def randomWalk(self, idx):
        new_direction = drawNormal(origin=self.p_manager.get_dir(idx))

        factor = self.EPSILON * 1
        x, y = self.p_manager.get_pos(idx)
        x += math.cos(new_direction) * factor
        y += math.sin(new_direction) * factor

        trying_this = self.p_manager.new_pos(
            idx=idx, pos=(x, y), dir=new_direction)
        return trying_this

    def getNextNode(self):
        if random.random() < 0:
            # if self.prob_vector_normalized is None or random.random() < 0.01:
            print('rand')
            p = self.randomSampler.getNextNode()
            choice = -1
        else:
            # get a node to random walk
            prob = self.p_manager.get_prob()
            self._last_prob = prob  # this will be used to paint particles
            choice = np.random.choice(range(self.p_manager.size()), p=prob)

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

    def get_color_transists(self, value, max_prob, min_prob):
        denominator = max_prob - min_prob
        if denominator == 0:
            denominator = 1  # prevent division by zero
        return 220 - 220 * (1 - (value - min_prob) / denominator)

    def paint(self, window):
        max = self._last_prob.max()
        min = self._last_prob.min()
        for i, p in enumerate(self.p_manager.particles):
            self.particles_layer.fill((255, 128, 255, 0))
            # get a transistion from green to red
            c = self.get_color_transists(self._last_prob[i], max, min)
            color = (100, c, 0)

            pygame.draw.circle(self.particles_layer, color,
                               p.pos * self.scaling, 4 * self.scaling)
            window.blit(self.particles_layer, (0, 0))
