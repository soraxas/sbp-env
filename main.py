import logging
import sys

import rrtstar
from likelihoodPolicySampler import LikelihoodPolicySampler
from particleFilterSampler import ParticleFilterSampler
from nearbyPolicySampler import NearbyPolicySampler
from mouseSampler import MouseSampler
from disjointTree import DisjointParticleFilterSampler
from randomPolicySampler import RandomPolicySampler

LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)
# INFO includes only loading
# DEBUG includes all outputs
ch = logging.StreamHandler(sys.stdout)
ch.setFormatter(logging.Formatter('%(message)s'))
# ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
LOGGER.addHandler(ch)

def main():
    epsilon = 10.0
    max_number_nodes = 300000
    goal_radius = 15
    prob_block_size = 5
    SCALING = 4
    IGNORE_STEP_SIZE = False

    sampler = NearbyPolicySampler(prob_block_size=prob_block_size)
    sampler = LikelihoodPolicySampler(prob_block_size=prob_block_size)
    sampler = RandomPolicySampler()
    sampler = MouseSampler()
    sampler = ParticleFilterSampler()
    sampler = DisjointParticleFilterSampler()

    rrt = rrtstar.RRT(
        showSampledPoint=True,
        scaling=SCALING,
        sampler=sampler,
        goalBias=False,
        image='map.png',
        epsilon=epsilon,
        max_number_nodes=max_number_nodes,
        radius=goal_radius,
        ignore_step_size=IGNORE_STEP_SIZE,
        always_refresh=False
        )
    try:
        rrt.run()
        # rrt.wait_for_exit()
        import pygame
        pygame.image.save(rrt.window,"screenshot.jpg")
    # except SystemExit:
    #     pass
    except Exception as e:
        LOGGER.error("==============================")
        LOGGER.exception("Exception occured: {}".format(e))
        LOGGER.error("==============================")
        LOGGER.error("Waiting to be exit...")
        rrt.wait_for_exit()

if __name__ == '__main__':
    main()
