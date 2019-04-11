import logging
import math
import random

import numpy as np
import pygame
from overrides import overrides

from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler
from randomness import NormalRandomnessManager

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

LOGGER = logging.getLogger(__name__)

ENERGY_MIN = 0
ENERGY_MAX = 10
ENERGY_START = 7
ENERGY_COLLISION_LOSS = 1

RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 1.5
RANDOM_RESTART_EVERY = 30
RESAMPLE_RESTART_EVERY = 0 # 200


class ParticleManager:
    def __init__(self, num_particles, startPt, goalPt, args):
        self.num_particles = num_particles
        self.init_energy()
        self.particles = []
        self.local_samplers_to_be_rstart = []
        self.goalPt = goalPt
        self.args = args

        for _ in range(self.num_particles):
            self.particles.append(
                Particle(
                    direction=random.uniform(0, math.pi * 2),
                    pos=startPt))

    def add_to_restart(self, lsampler):
        if lsampler not in self.local_samplers_to_be_rstart:
            self.local_samplers_to_be_rstart.append(lsampler)

    def init_energy(self):
        self.particles_energy = np.ones(self.num_particles)
        self.particles_energy *= ENERGY_START
        self.resync_prob()

    def size(self):
        return self.num_particles

    def modify_energy(self, idx=None, particle_ref=None, factor=None, set_val=None):
        # TODO: sometimes the keep tracking might go out of sync (and cause error in np.random.choice. Investigate this)
        # keep track how much energy this operation would modify,
        # so we can change the energy_sum accordingly
        if idx is None:
            # get idx from particle ref
            try:
                idx = self.particles.index(particle_ref)
            except ValueError:
                return
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
                    self.particles_energy[idx] = max(0, self.particles_energy[idx] - ENERGY_COLLISION_LOSS)
        else:
            raise Exception("Nothing set in modify_energy")

        delta = self.particles_energy[idx] - old_energy
        self.cur_energy_sum += delta

    def confirm(self, idx, pos):
        self.particles[idx].confirm(pos)

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
        self.args.env.stats.lscampler_restart_counter += 1
        min_idx = np.argmin(self.particles_energy)
        p = self.particles_energy[min_idx]
        randomPt = self.args.env.nodes[random.randint(0, len(self.args.env.nodes) - 1)].pos
        self.particles[min_idx] = Particle(pos=randomPt)
        self.modify_energy(min_idx, set_val=ENERGY_START)
        return p

    def random_restart_specific_value(self):
        """
        Restart all the particles that has < energy
        than a specified amount, to a random location
        based on existing tree nodes.
        """
        self.args.env.stats.lscampler_restart_counter += 1
        tmp = []
        for i in range(self.size()):
            if self.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                tmp.append(self.particles_energy[i])
                randomPt = self.args.planner.nodes[random.randint(0, len(self.args.planner.nodes) - 1)].pos
                self.particles[i] = Particle(pos=randomPt)
                self.modify_energy(i, set_val=ENERGY_START)
        return tmp

    def new_pos_in_free_space(self):
        """Return a particle that is in free space (from map)"""
        self.args.env.stats.lscampler_restart_counter += 1
        while True:
            new_p = random.random() * self.args.env.XDIM, random.random() * self.args.env.YDIM
            self.args.env.stats.add_sampled_node(new_p)
            if self.args.env.collides(new_p):
                self.args.env.stats.add_invalid(obs=True)
            else:
                self.args.env.stats.add_free()
                break
        return new_p

    def random_free_space_restart(self):
        """
        Restart all the particles that has < energy
        than a specified amount, to a random location
        in the map that is free.
        Might not work well for non-disjoint tree.
        """
        tmp = []
        for i in range(self.size()):
            if self.particles_energy[i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                tmp.append(self.particles_energy[i])
                randomPt = self.new_pos_in_free_space()
                self.particles[i] = Particle(pos=randomPt)
                self.modify_energy(i, set_val=ENERGY_START)
        return tmp

    def weighted_resampling(self):
        """
        Resampling to the same amount of particles than it was,
        based on the current particles' energy/weighting
        """
        self.args.env.stats.lscampler_restart_counter += 1
        prob = self.get_prob()
        new_particles = []
        for _ in range(self.size()):
            choice = np.random.choice(range(self.size()), p=prob)
            new_particles.append(Particle(pos=self.particles[choice].pos))
        self.particles[:] = new_particles
        self.init_energy()


############################################################
##                       Particles                        ##
############################################################
class Particle:
    def __init__(self, direction=None, pos=None):
        self.restart(direction=direction, pos=pos)

    def restart(self, direction=None, pos=None):
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

    def try_new_pos(self, new_pos, new_dir):
        # pos is a 2 index list-type object
        # Do nothing with new_pos for now as we confirm our final location via callback
        #################################################################################
        # new_dir is a scalar (for now TODO make it to general dimension later)
        self._trying_this_dir = new_dir

    def confirm(self, pos):
        # to confirm the final location of newly added tree node
        self.pos = pos
        self.direction = self._trying_this_dir


############################################################
##                        Sampler                         ##
############################################################

class ParticleFilterSampler(Sampler):

    @overrides
    def __init__(self, supressVisitedArea=True):
        self.supressVisitedArea = supressVisitedArea
        self._last_prob = None

        self.counter = 0
        self._c_random = 0
        self._c_resample = 0

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        # For benchmark stats tracking
        self.args.env.stats.lscampler_restart_counter = 0
        self.args.env.stats.lscampler_randomwalk_counter = 0

        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)
        self.randomnessManager = NormalRandomnessManager()
        # probability layer
        self.particles_layer = pygame.Surface(
            (self.args.XDIM * self.args.scaling, self.args.YDIM * self.args.scaling),
            pygame.SRCALPHA)

        self.p_manager = ParticleManager(num_particles=16,
                                         startPt=self.start_pos,
                                         goalPt=self.goal_pos,
                                         args=self.args)

    @overrides
    def report_fail(self, idx, **kwargs):
        if idx >= 0:
            self.p_manager.modify_energy(idx=idx, factor=0.7)

    @overrides
    def report_success(self, idx, **kwargs):
        self.p_manager.confirm(idx, kwargs['pos'])
        self.p_manager.modify_energy(idx=idx, factor=1)

    def randomWalk(self, idx):
        self.args.env.stats.lscampler_randomwalk_counter +=1
        # Randomly bias toward goal direction
        if random.random() < self.args.goalBias:
            dx = self.goal_pos[0] - self.p_manager.get_pos(idx)[0]
            dy = self.goal_pos[1] - self.p_manager.get_pos(idx)[1]
            goal_direction = math.atan2(dy, dx)
            new_direction = self.randomnessManager.draw_normal(origin=goal_direction, kappa=1.5)
        else:
            new_direction = self.randomnessManager.draw_normal(origin=self.p_manager.get_dir(idx), kappa=1.5)

        # scale the half norm by a factor of epsilon
        # Using this: https://docs.scipy.org/doc/scipy-0.15.1/reference/generated/scipy.stats.halfnorm.html
        # factor = self.randomnessManager.draw_half_normal(self.args.epsilon, scale=self.args.epsilon * 0.5)
        factor = self.args.epsilon
        x, y = self.p_manager.get_pos(idx)
        x += math.cos(new_direction) * factor
        y += math.sin(new_direction) * factor

        self.p_manager.new_pos(idx=idx,
                               pos=(x, y),
                               dir=new_direction)
        return (x, y)

    def get_random_choice(self):
        prob = self.p_manager.get_prob()
        self._last_prob = prob  # this will be used to paint particles
        try:
            choice = np.random.choice(range(self.p_manager.size()), p=prob)
        except ValueError as e:
            # NOTE dont know why the probability got out of sync... We notify the use, then try re-sync the prob
            LOGGER.error("!! probability got exception '{}'... trying to re-sync prob again.".format(e))
            self.p_manager.resync_prob()
            prob = self.p_manager.get_prob()
            self._last_prob = prob
            choice = np.random.choice(range(self.p_manager.size()), p=prob)
        return choice

    @overrides
    def get_next_pos(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1

        # if self._c_random > RANDOM_RESTART_EVERY and RANDOM_RESTART_EVERY > 0:
        #     _p = self.p_manager.random_restart_lowest()
        #     print("Rand restart at counter {}, with p {}".format(self.counter, _p))
        #     self._c_random = 0
        if self._c_random > RANDOM_RESTART_EVERY > 0:
            _p = self.p_manager.random_restart_specific_value()
            if _p:
                LOGGER.debug("Rand restart at counter {}, with p {}".format(self.counter, _p))
            self._c_random = 0
            self.p_manager.weighted_resampling()
            LOGGER.debug("Resampling at counter {}".format(self.counter))
            self._c_resample = 0
            LOGGER.debug(self.p_manager.get_prob())


        if random.random() < 0:
            LOGGER.debug('rand')
            p = self.randomSampler.get_next_pos()
            choice = -1
        else:
            # get a node to random walk
            choice = self.get_random_choice()

            p = self.randomWalk(choice)

        self.last_particle = p
        return (p, lambda c=choice, **kwargs: self.report_success(c, **kwargs),
                   lambda c=choice, **kwargs: self.report_fail(c, **kwargs))


############################################################
##                      FOR PAINTING                      ##
############################################################

    @staticmethod
    def get_color_transists(value, max_prob, min_prob):
        denominator = max_prob - min_prob
        if denominator == 0:
            denominator = 1  # prevent division by zero
        return 220 - 180 * (1 - (value - min_prob) / denominator)

    @overrides
    def paint(self, window):
        if self._last_prob is None:
            return
        max_num = self._last_prob.max()
        min_num = self._last_prob.min()
        for i, p in enumerate(self.p_manager.particles):
            self.particles_layer.fill((255, 128, 255, 0))
            # get a transition from green to red
            c = self.get_color_transists(self._last_prob[i], max_num, min_num)
            c = max(min(255, c), 50)
            color = (c, c, 0)
            self.args.env.draw_circle(pos=p.pos, colour=color, radius=4, layer=self.particles_layer)
            window.blit(self.particles_layer, (0, 0))
        ##### Texts
        # text = 'L.S.Walk:{}Res:{}'.format(self.rrt.stats.lscampler_randomwalk_counter, self.rrt.stats.lscampler_restart_counter)
        # window.blit(self.rrt.myfont.render(text, False, Colour.black, Colour.white), (self.rrt.XDIM * self.rrt.SCALING * 0.5, self.rrt.YDIM * self.rrt.SCALING * 0.95))
