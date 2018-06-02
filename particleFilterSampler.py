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
    - ✔ Bias toward goalPt
    - ✔ Using von mises distribution
    - Wall following?
    - RESTART
        - ✔ Random tree node restart (lowest energy)
        - ✔ Random tree node restart (particles with energy < specified amount)
        - ✔ Re-sampling according to particle weight/energy
        - Random free space restart (NEED TO CONNECT DIFFERENT TREES TOGETHER)
    - Keep structure of undelying map to restart


Use linear gradient
    |            /
    |          /
    |        /
    |      /
    |    /
    |  /
    |/________________
Use a factor of diff
    |-  -  -  -  -  -  -  -  -
    |                     _____
    |                ____/
    |           ____/
    |       __/
    |    _/
    |  /
    |/_______________________


"""

GOAL_BIAS = 0.05

ENERGY_MIN = 0
ENERGY_MAX = 10
ENERGY_START = 5

RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 1.5
RANDOM_RESTART_EVERY = 30
RESAMPLE_RESTART_EVERY = 0 # 200

############################################################
##                       Particles                        ##
############################################################

def drawNormal(origin, use_vonmises=True, kappa=1):
    if ('normal_dist_draws_reserve' not in drawNormal.__dict__
            or drawNormal.cur_idx + 2 >=
            drawNormal.normal_dist_draws_reserve.size):
        # redraw
        drawNormal.cur_idx = 0
        if use_vonmises:
            mu = 0
            dist = np.random.vonmises(mu, kappa, 1000)
        else:
            mu, sigma = 0, math.pi / 4
            dist = np.random.normal(mu, sigma, 1000)
        drawNormal.normal_dist_draws_reserve = dist
    # draw from samples
    draws = drawNormal.normal_dist_draws_reserve[drawNormal.cur_idx]
    drawNormal.cur_idx += 1
    # shift location
    draws += origin
    return draws


class ParticleManager:
    def __init__(self, num_particles, startPt, goalPt, nodes):
        self.num_particles = num_particles
        self.init_energy()
        self.particles = []
        self.goalPt = goalPt
        self.nodes = nodes

        for _ in range(self.num_particles):
            self.particles.append(
                Particle(
                    direction=random.uniform(0, math.pi * 2),
                    pos=startPt))

    def init_energy(self):
        self.particles_energy = np.ones(self.num_particles)
        self.particles_energy *= ENERGY_START
        self.resync_prob()

    def size(self):
        return self.num_particles

    def modify_energy(self, idx, factor=None, set_val=None):
        # TODO: sometimes the keep tracking might go out of sync (and cause error in np.random.choice. Investigate this)
        # keep track how much energy this operation would modify,
        # so we can change the energy_sum accordingly
        old_energy = self.particles_energy[idx]
        if set_val is not None:
            self.particles_energy[idx] = set_val
        elif factor is not None:
            if False:
                # NOTE WE ARE NOT DOING THIS FOR NOW
                self.particles_energy[idx] *= factor
            else:
                # TODO: mayne redo this nasty
                factor -= 1
                if factor > 0:
                    diff = ENERGY_MAX - self.particles_energy[idx]
                    self.particles_energy[idx] += diff*factor
                elif factor < 0:
                    diff = self.particles_energy[idx] - ENERGY_MIN
                    self.particles_energy[idx] += diff*factor

                    self.particles_energy[idx] -=1

        else:
            raise Exception("Nothing set in modify_energy")

        delta = self.particles_energy[idx] - old_energy
        self.cur_energy_sum += delta

    def confirm(self, idx):
        self.particles[idx].confirm()

    def new_pos(self, idx, pos, dir):
        return self.particles[idx].try_new_pos((pos[0], pos[1]), dir)

    def get_pos(self, idx):
        return self.particles[idx].pos

    def get_dir(self, idx):
        return self.particles[idx].direction

    def get_prob(self):
        return self.particles_energy / self.cur_energy_sum

    def resync_prob(self):
        self.cur_energy_sum = self.particles_energy.sum()

    def random_restart_lowest(self):
        """
        Restart the particle with the lowest energy.
        """
        min_idx = np.argmin(self.particles_energy)
        p = self.particles_energy[min_idx]
        randomPt = self.nodes[random.randint(0, len(self.nodes)-1)].pos
        self.particles[min_idx] = Particle(pos=randomPt)
        self.modify_energy(min_idx, set_val=ENERGY_START)
        return p

    def random_restart_specific_value(self):
        """
        Restart all the particles that has < energy
        than a specified amount.
        """
        tmp = []
        for i in range(self.size()):
            if self.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                tmp.append(self.particles_energy[i])
                randomPt = self.nodes[random.randint(0, len(self.nodes)-1)].pos
                self.particles[i] = Particle(pos=randomPt)
                self.modify_energy(i, set_val=ENERGY_START)
        return tmp

    def weighted_resampling(self):
        """
        Resampling to the same amount of particles than it was,
        based on the current particles' energy/weighting
        """
        prob = self.get_prob()
        new_particles = []
        for _ in range(self.size()):
            choice = np.random.choice(range(self.size()), p=prob)
            new_particles.append(Particle(pos=self.particles[choice].pos))
        self.particles[:] = new_particles
        self.init_energy()




class Particle:
    def __init__(self, direction=None, pos=None):
        if direction is None:
            # I will generate one if you dont give me!
            direction = random.uniform(0, math.pi * 2)
        if pos is None:
            # I cant really get started...can i?
            raise Exception("No pos given")
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


############################################################
##                        Sampler                         ##
############################################################

class ParticleFilterSampler:
    def __init__(self, prob_block_size, supressVisitedArea=True):
        self.PROB_BLOCK_SIZE = prob_block_size
        self.supressVisitedArea = supressVisitedArea
        self._last_prob = None

        self.counter = 0
        self._c_random = 0
        self._c_resample = 0

    def init(self, **kwargs):
        self.XDIM = kwargs['XDIM']
        self.YDIM = kwargs['YDIM']
        self.RRT = kwargs['RRT']
        self.EPSILON = kwargs['EPSILON']
        self.scaling = kwargs['SCALING']
        self.startPt = kwargs['startPt']
        self.goalPt = kwargs['goalPt']
        self.nodes = kwargs['nodes']
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(XDIM=self.XDIM, YDIM=self.YDIM, RRT=self.RRT)
        # probability layer
        self.particles_layer = pygame.Surface(
            (self.XDIM * self.scaling, self.YDIM * self.scaling),
            pygame.SRCALPHA)

        self.p_manager = ParticleManager(num_particles=10,
                                         startPt=self.startPt,
                                         goalPt=self.goalPt,
                                         nodes=self.nodes)

    def reportFail(self, idx):
        if idx >= 0:
            self.p_manager.modify_energy(idx=idx, factor=0.8)

    def reportSuccess(self, idx):
        self.p_manager.confirm(idx)
        self.p_manager.modify_energy(idx=idx, factor=1.1)

    def randomWalk(self, idx):
        # Randomly bias toward goal direction
        if random.random() < GOAL_BIAS:
            dx = self.goalPt[0] - self.p_manager.get_pos(idx)[0]
            dy = self.goalPt[1] - self.p_manager.get_pos(idx)[1]
            goal_direction = math.atan2(dy, dx)
            new_direction = drawNormal(origin=goal_direction, kappa=1.5)
        else:
            new_direction = drawNormal(origin=self.p_manager.get_dir(idx), kappa=1.5)

        factor = self.EPSILON * 1
        x, y = self.p_manager.get_pos(idx)
        x += math.cos(new_direction) * factor
        y += math.sin(new_direction) * factor

        trying_this = self.p_manager.new_pos(
            idx=idx, pos=(x, y), dir=new_direction)
        return trying_this

    def getNextNode(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1

        # if self._c_random > RANDOM_RESTART_EVERY and RANDOM_RESTART_EVERY > 0:
        #     _p = self.p_manager.random_restart_lowest()
        #     print("Rand restart at counter {}, with p {}".format(self.counter, _p))
        #     self._c_random = 0
        if self._c_random > RANDOM_RESTART_EVERY and RANDOM_RESTART_EVERY > 0:
            _p = self.p_manager.random_restart_specific_value()
            if _p:
                print("Rand restart at counter {}, with p {}".format(self.counter, _p))
            self._c_random = 0
        if self._c_resample > RESAMPLE_RESTART_EVERY and RESAMPLE_RESTART_EVERY > 0:
            print(self.p_manager.get_prob())
            self.p_manager.weighted_resampling()
            print("Resampling at counter {}".format(self.counter))
            self._c_resample = 0
            print(self.p_manager.get_prob())


        if random.random() < 0:
            print('rand')
            p = self.randomSampler.getNextNode()
            choice = -1
        else:
            # get a node to random walk
            prob = self.p_manager.get_prob()
            self._last_prob = prob  # this will be used to paint particles
            try:
                choice = np.random.choice(range(self.p_manager.size()), p=prob)
            except ValueError as e:
                # NOTE dont know why the probability got out of sync... We notify the use, then try re-sync the prob
                print("!! probability got exception '{}'... trying to re-sync prob again.".format(e))
                self.p_manager.resync_prob()
                self._last_prob = prob
                choice = np.random.choice(range(self.p_manager.size()), p=prob)

            p = self.randomWalk(choice)

        self.last_particle = p
        return p, choice

    def addTreeNode(self, x, y):
        return

    def addSampleLine(self, x1, y1, x2, y2):
        return

    def addSample(self, **kwargs):
        return


############################################################
##                      FOR PAINTING                      ##
############################################################

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
                               p.pos.astype(int) * self.scaling, 4 * self.scaling)
            window.blit(self.particles_layer, (0, 0))
